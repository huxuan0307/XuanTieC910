package Core.IFU

import Core.Config
import chisel3._
import chisel3.util._

//todo:这里只考虑64位ramhelper
class prefetch_bufferIO extends Bundle with Config with CacheConfig{
  //  val tag = Input(UInt(TagBits.W)) //
  //  val offset = Input(UInt(2.W))
  val vaddr = Input(UInt(PC_WIDTH.W))
  val paddr = Input(UInt(PC_WIDTH.W))
  val prefetch_valid = Input(Bool())
  //val prefetch_data = Input(Vec(4,UInt(128.W)))
  val prefetch_addr = Input(UInt(TagBits.W))
  val dout = ValidIO(Output(UInt(ICacheRespBits.W)))
}

class prefetch_buffer extends Module with Config with CacheConfig  {
  def ramhelper_width = 64
  val io = IO(new prefetch_bufferIO)
  val tag_array = Seq.fill(Ways)(RegInit(0.U(TagBits.W)))
  val valid_array = Seq.fill(Ways)(RegInit(false.B))
  val data_array = Seq.fill(Ways)(RegInit(Vec(CacheCatNum,0.U(CacheCatBits.W))))
  val fifo = RegInit(0.U(1.W))
  //计划设计为两行2*64byte，否则逻辑麻烦点
  val tagHitVec = Wire(Vec(Ways,Bool()))
  val tag = ftag(io.paddr)
  val index = findex(io.vaddr)
  val offset = index(2,1)//相当于PC前四位不看，从第5位开始，每变1 offset，就是下个128bit

  val prefetch_idx = Wire(UInt(PC_WIDTH.W))
  val prefetch_tag = ftag(io.prefetch_addr)

  val icache_refill = Module(new ct_ifu_icache_refill)


  val refill_ready = Wire(Bool())
  val refill_data = WireInit(Vec(CacheCatNum,0.U(CacheCatBits.W)))

  val state: UInt = RegInit(s_idle)
  val s_idle :: s_wait_refill :: s_refill :: Nil = Enum(3)
  //replace等待axi4响应，如果是ramhelper一次64bit数据，一个缓存块64byte需要8次传输

  icache_refill.io.req.valid := io.prefetch_valid
  icache_refill.io.req.bits.paddr := io.prefetch_addr
  refill_ready := icache_refill.io.resp.rvalid
  refill_data  := icache_refill.io.resp.rdata


  for (i <- 0 until Ways) {
    tagHitVec(i) := io.dout.valid && valid_array(i) && tag_array(i) === tag
  }
  val hit = Wire(Bool())
  hit := Mux(fifo===0.U(1.W),tagHitVec(0),tagHitVec(1)) //因为是逐路读写

  when(state===s_idle && hit){
    io.dout.valid := true.B
  }.otherwise{
    io.dout.valid := false.B
  }
  io.dout.bits := Mux(fifo===0.U(1.W),data_array(0).asUInt() >> (offset << 7.U),data_array(1).asUInt() >> (offset << 7.U))//<<7.U即 *128.U

  for( i<- 0 until  CacheCatNum){//这里先按C910考虑sram一次读写128bit
    when(state === s_refill){
      when(fifo===0.U){
        tag_array(0) := prefetch_tag //下一拍生效，生效同时转为idle
        valid_array(0) := true.B
        data_array(0) := refill_data.asTypeOf(data_array(0))//下一拍生效，生效同时转为idle
      }.otherwise{
        tag_array(1) := prefetch_tag
        valid_array(1) := true.B
        data_array(1) := refill_data.asTypeOf(data_array(1))
      }
    }
  }


  //-------------------------------------状态机------------------------------------------------
  switch(state) {
    is(s_idle) {
      when(io.prefetch_valid) {
        state := s_wait_refill
      }
    }
    is(s_wait_refill) {
      when(refill_ready) { //什么是非阻塞式缓存，是否有必要每拍写入一部分数据，ans:应该不用，因为tag没变。
        state := s_refill
      }
    }
    is(s_refill) {
      state := s_idle
      fifo := ~fifo
    }
  }
}
