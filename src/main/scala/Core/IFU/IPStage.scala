package Core.IFU
import Utils.UIntToMask
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util.{Cat, _}

class IPStage extends Module with Config {
  val io = IO(new IPStageIO)

  val ip_data_valid = !io.ip_flush && io.icache_resp.valid && io.if_vld

//  val pc_mask = UIntToMask.rightmask(io.pc(3,1), 9)(8,1)
  val pc_mask = PriorityMux(Seq(
  (io.pc(3,1)==="b000".U) -> "b1111_1111".U,
  (io.pc(3,1)==="b001".U) -> "b1111_1110".U,
  (io.pc(3,1)==="b010".U) -> "b1111_1100".U,
  (io.pc(3,1)==="b011".U) -> "b1111_1000".U,
  (io.pc(3,1)==="b100".U) -> "b1111_0000".U,
  (io.pc(3,1)==="b101".U) -> "b1110_0000".U,
  (io.pc(3,1)==="b110".U) -> "b1100_0000".U,
  (io.pc(3,1)==="b111".U) -> "b1000_0000".U
))

  val inst_32 = Wire(Vec(8,Bool()))
  val pc      = Wire(Vec(8,UInt(VAddrBits.W)))
  for(i <- 0 until 8){
    inst_32(i) := io.icache_resp.bits.inst_data(i)(1,0) === "b11".U
    pc(i)      := Cat(io.pc(VAddrBits-1,4),i.U)
  }

  //last half inst
  val h0_valid     = RegInit(false.B)
  val h0_data      = RegInit(0.U(16.W))
  val h0_predecode = RegInit(0.U(4.W))
  val h0_pc        = RegInit(0.U(VAddrBits.W))
  val h0_br    = h0_valid && h0_predecode(2)
  val h0_ab_br = h0_valid && h0_predecode(3)
  h0_data      := RegNext(io.icache_resp.bits.inst_data(7))
  h0_predecode := RegNext(io.icache_resp.bits.predecode(7))
  h0_pc        := RegNext(pc(7))


  //predecode
  val bry0  = Wire(Vec(8,Bool()))
  val bry1  = Wire(Vec(8,Bool()))
  val br    = Wire(Vec(8,Bool()))
  val ab_br = Wire(Vec(8,Bool()))

  for(i <- 0 until 8) {
    bry0(i)  := io.icache_resp.bits.predecode(i)(0)
    bry1(i)  := io.icache_resp.bits.predecode(i)(1)
    br(i)    := io.icache_resp.bits.predecode(i)(2) //branch and jal (get offset)
    ab_br(i) := io.icache_resp.bits.predecode(i)(3) //jal
  }

  //bry1_hit是数学问题，想了很久，终于明白了。。。
  val bry1_hit = !h0_valid && bry1(io.pc(3,1))
  val bry = Mux(bry1_hit, bry1, bry0)

  //h0_valid update
  when(!io.ip_redirect.valid && !io.ubtb_resp.valid && bry(7) && inst_32(7) && ip_data_valid){
    h0_valid := true.B
  }.otherwise{
    h0_valid := false.B
  }

  //==========================================================
  //                   BHT Information
  //==========================================================
  //BHT Result Get
  val bht_pre_array  = Mux(io.bht_resp.pre_sel(1), io.bht_resp.pre_taken, io.bht_resp.pre_ntaken)
  val bht_pre_result = bht_pre_array(io.bht_resp.pre_offset)

  val br_taken = Wire(Vec(8,Bool()))
  val br_ntake = Wire(Vec(8,Bool()))

  br_taken(0) := (br(0) && pc_mask(0) && bry(0)) || h0_br
  br_ntake(0) := (ab_br(0) && pc_mask(0) && bry(0))

  br_taken(7) := (br(7) && pc_mask(7) && bry(7)) && !inst_32(7)
  br_ntake(7) := (ab_br(7) && pc_mask(7) && bry(7)) && !inst_32(7)

  for(i <- 1 until 7){
    br_taken(i) := br(i) && pc_mask(i) && bry(i)
    br_ntake(i) := ab_br(i) && pc_mask(i) && bry(i)
  }

  val br_mask = Mux(bht_pre_result(1), br_taken, br_ntake)

  //btb result
  val btb_sel = Wire(Vec(4,Bool()))
  for(i <- 0 until 4){
    btb_sel(i) := br_mask(2*i+1) || br_mask(2*i)
  }

  val btb_target = PriorityMux(Seq(
    btb_sel(0) -> io.btb_resp(0).bits,
    btb_sel(1) -> io.btb_resp(1).bits,
    btb_sel(2) -> io.btb_resp(2).bits,
    btb_sel(3) -> io.btb_resp(3).bits
  ))

  val btb_valid = PriorityMux(Seq(
    btb_sel(0) -> io.btb_resp(0).valid,
    btb_sel(1) -> io.btb_resp(1).valid,
    btb_sel(2) -> io.btb_resp(2).valid,
    btb_sel(3) -> io.btb_resp(3).valid
  ))

  //pred info
  val btb_miss     = br_mask.asUInt.orR && !btb_valid
  val ubtb_miss    = br_mask.asUInt.orR && !io.ubtb_resp.valid
  val ubtb_mispred = br_mask.asUInt.orR && io.ubtb_resp.valid && (btb_target =/= io.ubtb_resp.bits.target_pc(20,1) && !io.ubtb_resp.bits.is_ret)

  io.ip_redirect.valid := ip_data_valid && btb_valid && (ubtb_miss || ubtb_mispred)
  io.ip_redirect.bits  := Cat(io.pc(VAddrBits-1,21), btb_target, 0.U(1.W))

  //ipdecode
  val ipdecode = Module(new IPDecode)
  ipdecode.io.half_inst := h0_data +: io.icache_resp.bits.inst_data
  ipdecode.io.icache_br := h0_br +: br

  // pc mask h0 -->>9 bits mask
  val bry9     = Cat(bry.asUInt(),h0_valid)  // inst start info
  val pc_mask9 = Cat(pc_mask,h0_valid)
  val chgflw  = ipdecode.io.decode_info.chgflw.asUInt() & bry9 & Cat(!inst_32(7),"b1111_1111".U)  //contain all the chgflw inst except con_br
  val con_br  = ipdecode.io.decode_info.con_br.asUInt() & bry9 & Cat(!inst_32(7),"b1111_1111".U)
  val pcall   = ipdecode.io.decode_info.call.asUInt() & bry9 & Cat(!inst_32(7),"b1111_1111".U)
  val preturn = ipdecode.io.decode_info.ret.asUInt() & bry9 & Cat(!inst_32(7),"b1111_1111".U)
  val jalr    = ipdecode.io.decode_info.jalr.asUInt() & bry9 & Cat(!inst_32(7),"b1111_1111".U)
  val jal     = ipdecode.io.decode_info.jal.asUInt() & bry9 & Cat(!inst_32(7),"b1111_1111".U)
  val dst     = ipdecode.io.decode_info.dst_vld.asUInt() & bry9 & Cat(!inst_32(7),"b1111_1111".U)
  val chgflw_after_head = Mux(bht_pre_result(1), chgflw | con_br, chgflw) & pc_mask9 //take bht predict con_br result
  val chgflw_mask_pre = PriorityMux(Seq(
    chgflw_after_head(0) -> "b0000_0001_1".U,
    chgflw_after_head(1) -> Mux(inst_32(0),"b0000_0011_1".U,"b0000_0001_1".U),
    chgflw_after_head(2) -> Mux(inst_32(1),"b0000_0111_1".U,"b0000_0011_1".U),
    chgflw_after_head(3) -> Mux(inst_32(2),"b0000_1111_1".U,"b0000_0111_1".U),
    chgflw_after_head(4) -> Mux(inst_32(3),"b0001_1111_1".U,"b0000_1111_1".U),
    chgflw_after_head(5) -> Mux(inst_32(4),"b0011_1111_1".U,"b0001_1111_1".U),
    chgflw_after_head(6) -> Mux(inst_32(5),"b0111_1111_1".U,"b0011_1111_1".U),
    chgflw_after_head(7) -> Mux(inst_32(6),"b1111_1111_1".U,"b0111_1111_1".U),
    chgflw_after_head(8) -> "b1111_1111_1".U,
    true.B               -> "b1111_1111_1".U
  ))
  val chgflw_vld_mask = chgflw_mask_pre & pc_mask9
  val con_br_vld  = chgflw_vld_mask & con_br
  val pcall_vld   = chgflw_vld_mask & pcall
  val preturn_vld = chgflw_vld_mask & preturn
  val ind_vld     = chgflw_vld_mask & jalr & !preturn
  val jal_vld     = chgflw_vld_mask & jal
  val jalr_vld    = chgflw_vld_mask & jalr
  val push_pc_vec = Wire(Vec(8+1,UInt(VAddrBits.W)))
  push_pc_vec(0) := Cat(io.pc(VAddrBits-1,4), 0.U(4.W)) + 2.U  //inst 32
  push_pc_vec(8) := Cat(io.pc(VAddrBits-1,4), 0.U(4.W)) + 16.U // inst 16
  for(i <- 1 until 8){
    push_pc_vec(i) := Cat(io.pc(VAddrBits-1,4), 0.U(4.W)) + Mux(inst_32(i-1), 2.U, 0.U) + (i.U << 1.U)
  }
  // take 9 pc where is pc?
  val push_pc = PriorityMux(Seq(
    pcall_vld(0) -> push_pc_vec(0),
    pcall_vld(1) -> push_pc_vec(1),
    pcall_vld(2) -> push_pc_vec(2),
    pcall_vld(3) -> push_pc_vec(3),
    pcall_vld(4) -> push_pc_vec(4),
    pcall_vld(5) -> push_pc_vec(5),
    pcall_vld(6) -> push_pc_vec(6),
    pcall_vld(7) -> push_pc_vec(7),
    pcall_vld(8) -> push_pc_vec(8)
  ))

  val br_position = PriorityMux(Seq(
    br_mask(0) -> 0.U,
    br_mask(1) -> 1.U,
    br_mask(2) -> 2.U,
    br_mask(3) -> 3.U,
    br_mask(4) -> 4.U,
    br_mask(5) -> 5.U,
    br_mask(6) -> 6.U,
    br_mask(7) -> 7.U
  ))

  val br_offset = WireInit(0.U(21.W))
  for(i <- 0 until 8){
    when(br_position === i.U) {
      br_offset := ipdecode.io.decode_info.offset(i)
    }
  }
//  // offset
//  val branch_mask = Cat(br_mask.asUInt(),br_mask(0).asUInt()) & chgflw_vld_mask
//  val branch_base = PriorityMux(Seq(
//    branch_mask(0) -> push_pc_vec(0),
//    branch_mask(1) -> push_pc_vec(1),
//    branch_mask(2) -> push_pc_vec(2),
//    branch_mask(3) -> push_pc_vec(3),
//    branch_mask(4) -> push_pc_vec(4),
//    branch_mask(5) -> push_pc_vec(5),
//    branch_mask(6) -> push_pc_vec(6),
//    branch_mask(7) -> push_pc_vec(7),
//    branch_mask(8) -> push_pc_vec(8)
//  ))
//
//  io.br_res.ib_br_offset := br_offset
//  // base
//  io.br_res.ib_br_base := branch_base
//  //result
//  io.br_res.ib_br_result := Cat(io.pc(38,20),btb_target)
//  //index pc
//  val index_pc = PriorityMux(Seq(
//    br_mask(0) -> Cat(io.pc(38,3),"b000".U),
//    br_mask(1) -> Cat(io.pc(38,3),"b001".U),
//    br_mask(2) -> Cat(io.pc(38,3),"b010".U),
//    br_mask(3) -> Cat(io.pc(38,3),"b011".U),
//    br_mask(4) -> Cat(io.pc(38,3),"b100".U),
//    br_mask(5) -> Cat(io.pc(38,3),"b101".U),
//    br_mask(6) -> Cat(io.pc(38,3),"b110".U),
//    br_mask(7) -> Cat(io.pc(38,3),"b111".U),
//  ))
//  io.br_res.ib_btb_index_pc := index_pc
//  io.br_res.ib_ubtb_hit := ubtb_miss
  io.ip_bht_con_br_vld   := ip_data_valid && con_br_vld.orR()
  io.ip_bht_con_br_taken := bht_pre_result(1)

  //to IBStage
  io.out.valid := ip_data_valid
  io.out.bits.pc := io.pc
  io.out.bits.icache_resp := io.icache_resp.bits
  io.out.bits.decode_info := ipdecode.io.decode_info
  io.out.bits.is_ab_br := ab_br(br_position)
  io.out.bits.bht_resp := io.bht_resp
  io.out.bits.bht_res  := bht_pre_result
  io.out.bits.br_position := br_position
  io.out.bits.br_offset := br_offset
  io.out.bits.br_valid  := br_mask.asUInt().orR()
  io.out.bits.btb_valid := btb_valid
  io.out.bits.btb_target := btb_target
  io.out.bits.btb_miss := btb_miss
  io.out.bits.btb_sel := btb_sel.asUInt()
  io.out.bits.ubtb_valid := io.ubtb_resp.valid
  io.out.bits.ubtb_resp := io.ubtb_resp.bits
  io.out.bits.ubtb_miss := ubtb_miss
  io.out.bits.ubtb_mispred := ubtb_mispred
  io.out.bits.pcall := pcall_vld.orR
  io.out.bits.pret := preturn_vld.orR
  io.out.bits.ind_vld := ind_vld.orR
  io.out.bits.push_pc := push_pc
  io.out.bits.h0_vld       := h0_valid
  io.out.bits.h0_data      := h0_data
  io.out.bits.h0_predecode := h0_predecode
  io.out.bits.inst_32_9    := Cat(inst_32.asUInt(),1.U(1.W)) & bry9
  io.out.bits.chgflw_vld_mask := chgflw_vld_mask
  io.out.bits.h0_pc         := h0_pc
  io.out.bits.cur_pc        := pc

  io.out.bits.jal  := jal_vld(7,0)
  io.out.bits.jalr := jal_vld(7,0)
  io.out.bits.con_br := con_br_vld(7,0)
  io.out.bits.dst  := dst(7,0)
}
