package Core.IFU

import Core.Config
import chisel3._
import chisel3.util._


//sram一直在读写，如果命中，直接出去
//数据16bit一跳转，PC就忽略最低1位；32bit 2位；64bit 3位；128bit 4位
//todo:考虑分支指令带来的128 bit取值效率损失，与MMIO接口不用axi4试试
//先做PIPT
//C910 L1 ICache 一次返回128bit数据，在ip级进行处理，因此不必使用offset;在data_array里idx有11位，可用以作为4个128 bit偏移



//class ct_ifu_icache_if_io extends Bundle with Config with CacheConfig {
//  val icache_if_ifctrl_inst_data = Vec(Ways,Output(UInt(128.W)))
//  val icache_if_ifctrl_tag_data = Vec(Ways,Output(UInt(29.W)))
//}//C910采用虚拟地址作为index

class ICacheIO extends Bundle with Config with CacheConfig {//todo:SimpleBus里data返回是64位，改成128位或？
  //val bus = Flipped(new SimpleBus) // 包括去和返，流水线与缓存的通路（本是直接连往内存设备，现在经缓存中转），两条指令对应的MMIO的通用简单读写端口
  //val to_rw   = new AXI4IO   //缓存通往MMIO的总线，缓存是主机//采用单独的refill模块，定义refill接口
  //////val req = DecoupledIO(Input(UInt(PC_WIDTH.W)))//只需要ValidIO，取值pc更新根据icache_dout valid决定
  //////val paddr = Input(UInt(PC_WIDTH.W))
  //////val dout = ValidIO(Output(UInt(ICacheRespBits.W)))
  //////val dout_predec = Output(UInt(ICachePredecBits.W))
  val cache_req  = Flipped(DecoupledIO(new ICacheReq))
  val cache_resp = ValidIO(new ICacheResp)
  val cohreq = ValidIO(UInt(XLEN.W)) //coh，缓存一致性，连到dcache的请求，输出
  val cohresp = Flipped(ValidIO(new cohResp)) //dcahce返回的响应数据，输入
}

class ICache(cacheNum: Int =0, is_sim: Boolean) extends Module with Config with CacheConfig {
  val io = IO(new ICacheIO)


  //添加Prefetch Buffer，当refill请求后，判断有无命中，命中则使用，未命中丢弃；cache继续进行refill，prefetch buffer预取下一行
  //  val s_idle :: s_lookUp :: s_mmio :: s_mmio_resp :: s_miss :: s_replace :: s_refill :: s_refill_done :: Nil = Enum(8)
  val s_idle :: s_lookUp :: s_miss :: s_cohresp :: s_refill :: s_refill_done :: Nil = Enum(6)
  val state: UInt = RegInit(s_idle)


  val valid = RegInit(VecInit(Seq.fill(Ways)(VecInit(Seq.fill(Sets)(false.B)))))
  val fifo_array = RegInit(VecInit(Seq.fill(Sets)(0.U(1.W)))) //哪路为首先替换路
  val Way_read = RegInit(0.U(1.W))
  val Way_fill = RegInit(0.U(1.W))
  val predecd_array = Seq.fill(Ways)(Module(new sram_2048x32))//Seq.fill(Sets)(RegInit(VecInit(Seq.fill(Ways)(0.U(TagBits.W)))))
  val tag_array = RegInit(VecInit(Seq.fill(Ways)(VecInit(Seq.fill(Sets)(0.U(TagBits.W))))))
  val needRefill = RegInit(VecInit(Seq.fill(Ways)(VecInit(Seq.fill(Sets)(false.B)))))
  //  val missholdReg = RegInit(VecInit(Seq.fill(MSHRSize)(0.U(TagBits.W))))
  //  val misshold_vldReg = RegInit(VecInit(Seq.fill(MSHRSize)(0.U(CacheCatNum.W))))//用来记录miss填写状态
  //  val misshold_index = RegInit(0.U(log2Up(MSHRSize).W))
  val refill_ready = Wire(Bool())
  val refill_cnt = RegInit(0.U(log2Up(CacheCatNum+1).W))
  //val refill_data = RegInit(VecInit(Seq.fill(CacheCatNum)(0.U(CacheCatBits.W))))

  val reqreadReg  = RegInit(0.U(128.W))// 4*128bit //临时寄放读取Cache Sram的信号，C910一次读128bit
  val reqreadPredecReg = RegInit((0.U((ICachePredecBits).W)))//4*32bit

  val reqreadPredecWire = WireInit(0.U((ICachePredecBits).W))
  val SRam_read = WireInit(0.U(128.W))//Seq.fill(CacheCatNum)(WireInit(VecInit(Seq.fill(Banks)(0.U(sram_width.W)))))//4次取，bank表数目，每表32bit
  val SRam_write  = WireInit(VecInit(Seq.fill(CacheCatNum)(VecInit(Seq.fill(Banks)(0.U(sram_width.W))))))//4次取，bank表数目，每表32bit


  val paddr = io.cache_req.bits.paddr //////todo: check addrBits, 39 or 40
  val vaddr = io.cache_req.bits.vaddr //////todo: check addrBits, 39 or 40
  val tag = ftag(paddr)
  val index = findex(vaddr) //目前data_array、predecd_array采用与其它不同宽度的index
  val Dindex = fDindex(vaddr) //
  val storeEn     = WireInit(false.B)

  val reqValid    = RegInit(false.B) // Reg(Vec(2, Bool())) //用寄存器记录两条指令内存请求有效
  val paddrReg = RegInit(PcStart.U(PC_WIDTH.W))//RegEnable(paddr,RegNext(storeEn))//
  val vaddrReg = RegInit(PcStart.U(PC_WIDTH.W))//RegEnable(vaddr,RegNext(storeEn))//
  val addroffsetReg = RegInit(0.U(PC_WIDTH.W))
  val tagReg = ftag(paddrReg)
  val indexReg = findex(vaddrReg)
  val DindexReg = fDindex(vaddrReg)
  val icache_ready = RegInit(true.B)
  val rvaddrReg = vaddrReg+addroffsetReg //////no offset when refill, but need when read
  val rDindexReg = fDindex(rvaddrReg)

  val tagHitVec = Wire(Vec(Ways,Bool()))
  val hit = Wire(Bool())
  val hitReg = RegInit(true.B)
  val axireadMemCnt = RegInit(0.U(log2Up(RetTimes+1).W))  //通过axi读内存设备的计数，受axi总线宽度和cache数据大小所限


  val refill_idx = indexReg
  val Drefill_idx = DindexReg

  //val validVec =  Seq.fill(Ways)(Wire(Bool()))

  val refillData = WireInit(VecInit(Seq.fill(CacheCatNum)(0.U(axiDataBits.W))))  //用来寄存读取内存的数据, 4*128bit, 64 byte
  //val refillDataReg = WireInit(VecInit(Seq.fill(CacheCatNum)(0.U(axiDataBits.W))))

  val mem_wb = Seq.fill(CacheCatNum)(Wire(Vec(Banks, UInt(sram_width.W)))) //一次取4个bank每个bank 32bit, 128bit, 取四次填满
  val DreadIdx = Wire(UInt(DIndexBits.W)) //for data sram only
  val CohReadReg = RegInit(VecInit(Seq.fill(CacheCatNum)(0.U(axiDataBits.W)))) //dcache给出的数据，参与refill，因此给64byte//todo:check it
  val stateReg2 = RegInit((state === s_cohresp && io.cohresp.valid && !io.cohresp.bits.needforward))
  val hit_read = state===s_refill_done || io.cache_req.valid
  val fw_write = state === s_miss && io.cohresp.valid && io.cohresp.bits.needforward //其它一致性缓存继续写cache
  val refill_write = state === s_refill //&& axireadMemCnt === RetTimes.U //完成写的那拍
  val SRamArray    = Seq.fill(Ways)(Module(new sram_c910)) //构建128bit宽度sram, 4个bank, 例化两个代表两路
  val icache_refill = Module(new ct_ifu_icache_refill)//todo: check it

//  val wb_paddr = RegInit(0.U(PC_WIDTH.W))
//  val wb_data = Reg


  val icache_predec = Module(new icache_predec)
  val idle_afterfill = (RegNext(state===s_refill_done) && (state=== s_idle))
  val lookup_afteridle = (RegNext(state=== s_lookUp)&& state===s_idle)



  storeEn := (state === s_idle || state === s_lookUp)//流水线请求有效并且处于idle或Cache访问请求寻址状态

  predecd_array(0).io := DontCare
  predecd_array(1).io := DontCare
  icache_predec.io := DontCare
  when(state===s_refill_done || state===s_idle || state===s_lookUp){
    predecd_array(0).io.idx := Mux(state === s_idle || state === s_lookUp, Dindex, rDindexReg)
    predecd_array(1).io.idx := Mux(state === s_idle || state === s_lookUp, Dindex, rDindexReg)
  }
  reqreadPredecWire := Mux(Way_read===0.U,predecd_array(0).io.rData,predecd_array(1).io.rData)
  reqreadPredecReg := reqreadPredecWire
  when(axireadMemCnt <= RetTimes.U && state === s_refill){
    icache_predec.io.din := SRam_write(RegNext(axireadMemCnt)).asUInt()

    predecd_array(0).io.idx := Mux(refill_write || fw_write, Drefill_idx, Dindex) + RegNext(axireadMemCnt)
    predecd_array(0).io.wMask := VecInit(Seq.fill(32)(true.B)).asUInt() //wmask had been done
    predecd_array(0).io.wData := icache_predec.io.dout
    predecd_array(0).io.en := (hit_read || refill_write || fw_write)
    predecd_array(0).io.wen :=  fw_write || refill_write && refill_ready && (Way_fill ===0.U)


    predecd_array(1).io.idx := Mux(refill_write || fw_write, Drefill_idx, Dindex)+ RegNext(axireadMemCnt)
    predecd_array(1).io.wMask := VecInit(Seq.fill(32)(true.B)).asUInt() //wmask had been done
    predecd_array(1).io.wData := icache_predec.io.dout
    predecd_array(1).io.en := (hit_read || refill_write || fw_write)
    predecd_array(1).io.wen := fw_write || refill_write && refill_ready && (Way_fill ===1.U)

  }


  for (i <- 0 until Ways) {
    tagHitVec(i) := io.cache_req.valid && valid(i)(index) && tag_array(i)(index) === tag
    //validVec(i) := valid(i)(indexReg)//似乎并没有用到
  }
  hit := tagHitVec.asUInt.orR //每一拍都直接辨别当拍是否hit
  hitReg := hit
  reqValid := io.cache_req.valid //当流水线请求任一条有效时，更新为refill准备的寄存器里的请求有效
  //fifo_read := Mux(state===s_idle || state===s_lookUp,~fifo_array(index),fifo_array(index))
  when(storeEn) {
    paddrReg := Cat(paddr(PAddrBits - 1,4),0.U(4.W))
    vaddrReg := Cat(vaddr(VAddrBits - 1,4),0.U(4.W))
    addroffsetReg := vaddr(3,0)
    Way_read := tagHitVec(1)
    when(io.cache_req.valid && !hit) {

    }
  }
  //  paddrReg := RegEnable(paddr,RegNext(storeEn))
  //  vaddrReg := RegEnable(vaddr,RegNext(storeEn))

  //miss后寻找其它L1缓存
  io.cohreq.valid := state === s_miss
  io.cohreq.bits  := paddrReg
  //s_mmio_resp
  //mmio_inst := io.cohresp.bits  //从其它L1缓存中取出的指令数据,当拍结束时可以立即给出去
  //
  for (i <- 0 until  CacheCatNum) {
    when(state === s_miss && io.cohresp.valid && io.cohresp.bits.needforward){ //cache miss并且响应有效，且响应需要向前
      tag_array(Way_fill)(refill_idx) := tagReg
      valid(Way_fill)(refill_idx)    := true.B
      CohReadReg(i) := io.cohresp.bits.forward(i).asTypeOf(UInt(axiDataBits.W))
      SRam_write(i) := io.cohresp.bits.forward(i).asTypeOf(Vec(Banks,UInt(sram_width.W)))
    }
  }
  when(state === s_miss){
    axireadMemCnt := 0.U(log2Up(RetTimes).W)  //还未进行refill，所以axi读内存的计数置为0.U
  }

  icache_refill.io.req.valid := (state === s_refill) //|| (state === s_mmio)//
  icache_refill.io.req.bits.paddr := Cat(paddrReg(39,4),0.U(4.W))//C910里一次读128bit数据，PC+1对应8bit，+16对应128，因而PC低4位在读128bit块时不需要考虑
  refill_ready := icache_refill.io.resp.rvalid

  //侦听axi响应4次,从下级缓存或内存所读取
  SRamArray(0).io := DontCare
  SRamArray(1).io := DontCare
  when(state===s_refill_done || state===s_idle || state===s_lookUp) {
    SRamArray(0).io.idx := Mux(state === s_idle || state === s_lookUp, Dindex, rDindexReg)
    SRamArray(1).io.idx := Mux(state === s_idle || state === s_lookUp, Dindex, rDindexReg)
  }
  SRam_read := Mux(Way_read === 0.U,SRamArray(0).io.rData.asTypeOf(SRam_read),SRamArray(1).io.rData.asTypeOf(SRam_read))
  reqreadReg := SRam_read.asUInt() //读Cache Data寄存器更新，放到寄存器里
  when(axireadMemCnt <= RetTimes.U && state === s_refill){
    SRamArray(0).io.idx := Mux(refill_write || fw_write, Drefill_idx, Dindex)+ RegNext(axireadMemCnt)//Cat(Mux(refill_write || fw_write, Drefill_idx, Dindex)(9,2),0.U(2.W)) + RegNext(RegNext(axireadMemCnt<<2))
    SRamArray(0).io.wMask := VecInit(Seq.fill(128)(true.B)).asUInt() //wmask had been done
    SRamArray(0).io.wData := SRam_write(RegNext(axireadMemCnt)).asUInt()
    SRamArray(0).io.en := (hit_read || refill_write || fw_write)
    SRamArray(0).io.wen := fw_write || refill_write && refill_ready && (Way_fill===0.U)
    SRamArray(1).io.idx := Mux(refill_write || fw_write, Drefill_idx, Dindex)+ RegNext(axireadMemCnt)//Cat(Mux(refill_write || fw_write, Drefill_idx, Dindex)(9,2),0.U(2.W)) + RegNext(RegNext(axireadMemCnt<<2))
    SRamArray(1).io.wMask := VecInit(Seq.fill(128)(true.B)).asUInt() //wmask had been done
    SRamArray(1).io.wData := SRam_write(RegNext(axireadMemCnt)).asUInt()
    SRamArray(1).io.en := (hit_read || refill_write || fw_write)
    SRamArray(1).io.wen := fw_write || refill_write && refill_ready && (Way_fill ===1.U)

    when(needRefill(Way_fill)(indexReg) && refill_ready){
      refillData(axireadMemCnt) := icache_refill.io.resp.rdata
    }
    when(refill_ready){//每收到一次回应，axi读次数＋1
      axireadMemCnt := axireadMemCnt + 1.U(log2Up(RetTimes).W)
    }
  }

  //for refill done state hit match
  DreadIdx := DindexReg//Mux(state===s_refill_done,DindexReg,Dindex)

  for( i<- 0 until  CacheCatNum){//这里先按C910考虑sram一次读写128bit
    mem_wb(i) := Mux(stateReg2,CohReadReg(i).asUInt().asTypeOf(mem_wb(i)),
      refillData(i).asUInt().asTypeOf(mem_wb(i))) //用dcache数据进行refill,这样refill_done时统一预译码。refillDataReg 4*128 bit， 64 byte，一次refill的大小, mem_wb是4*4*32
    SRam_write(i) := mem_wb(i) //通过此连线将readDataReg数据格式转为sram所需
//    when(state === s_refill && axireadMemCnt === RetTimes.U){
//      tag_array(fifo_fill)(refill_idx) := tagReg
//      valid(fifo_fill)(refill_idx) := true.B
//    }.otherwise{
//      mem_wb(i) := DontCare
//    }
  }
  when(state === s_refill && axireadMemCnt === RetTimes.U){
      tag_array(Way_fill)(refill_idx) := tagReg
      valid(Way_fill)(refill_idx) := true.B
  }


  stateReg2 := (state === s_cohresp && io.cohresp.valid && !io.cohresp.bits.needforward)//state === s_mmio_resp  //每一拍都侦听是否mmio响应了//根据是refill_done还是Dcache响应决定输出数据来源
  io.cache_req.ready  :=  (state===s_idle) || (state===s_lookUp)//这里考虑可以一直读sram，即便在进行refill
  val Cohresp_valid = WireInit(false.B)
  Cohresp_valid := stateReg2 && state ===s_refill
  val Cohresp_data = CohReadReg.asUInt() >> (Dindex(1) << 7.U)
  val reqsramReg_data =  reqreadReg.asUInt()
  val reqsramWire_data = SRam_read.asUInt
  val reqsram_data = Mux(state===s_lookUp || idle_afterfill, reqsramWire_data, reqsramReg_data) //todo: check it
  val req_data = WireInit(0.U(128.W))
  req_data := Mux(Cohresp_valid, Cohresp_data, reqsram_data)
  io.cache_resp.bits.inst_data  := req_data.asTypeOf(io.cache_resp.bits.inst_data)//上一拍mmio响应了，这一拍idle，说明其它缓存响应有效，否则从readReg里加偏移量取数据
  //右移动偏移量Dindex(1,0)*128,用左移代替乘，数据自动截断？todo: change Bundle bus
  val predecode_out = Mux(state===s_lookUp || idle_afterfill,reqreadPredecWire,reqreadPredecReg)
  io.cache_resp.bits.predecode := predecode_out.asTypeOf(io.cache_resp.bits.predecode)//移动Dindex*32位
  io.cache_resp.valid := (state===s_lookUp) || idle_afterfill


  //-------------------------------------状态机------------------------------------------------
  switch(state) {
    is(s_idle) {
      when(!hit && io.cache_req.valid){
        Way_fill := fifo_array(index)
        needRefill(fifo_array(index))(index) := io.cache_req.valid && !hit
        state := s_miss
      }.elsewhen(io.cache_req.valid) {
        state := s_lookUp
      }
    }
    is(s_lookUp) {
      when(!hit && io.cache_req.valid ){
        Way_fill := fifo_array(index)
        needRefill(fifo_array(index))(index) := io.cache_req.valid && !hit
        state := s_miss
      }.elsewhen(!io.cache_req.valid){
        state := s_idle
      }
    }
    is(s_miss) {
      Way_read := Way_fill
      when(io.cohresp.valid || io.cohresp.bits.needforward){//todo:假定needforward保持为false,一进dcache若所需存在dcache立即置为true
        state := s_cohresp
      }.otherwise{
        state := s_refill
        valid(Way_fill)(refill_idx) := false.B
      }
    }
    is(s_cohresp){
      when(io.cohresp.valid) {
        when(io.cohresp.bits.needforward && needRefill(Way_fill)(refill_idx)) {
          state := s_cohresp
          needRefill(Way_fill)(refill_idx) := false.B //其它缓存响应了不需要refill了
        }.elsewhen(!io.cohresp.bits.needforward && needRefill(Way_fill)(refill_idx)) {
          needRefill(Way_fill)(refill_idx) := false.B //其它缓存响应了不需要refill了
          state := s_refill_done
        }
      }
    }
    is(s_refill) {
      when((axireadMemCnt === RetTimes.U) && needRefill(Way_fill)(refill_idx)){
        state := s_refill_done
        needRefill(Way_fill)(refill_idx) := false.B
        fifo_array(refill_idx) := ~Way_fill
        //fifo_array(refill_idx) := ~fifo
        //fifo_array(refill_idx) := fifo_fill
      }
    }
    is(s_refill_done){
      state := s_idle
    }
  }
}
