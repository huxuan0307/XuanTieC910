package Core.LSU.LoadExStage

import Core.LsuConfig
import Utils.sext
import chisel3._
import chisel3.util._

class LoadWBInput extends Bundle{
  val fromCp0 = new Bundle{
    val lsu_icg_en = Bool()
    val yy_clk_en = Bool()
  }
  val ctrl_ld_clk = Bool()
  val fromHad = new Bundle{
    val bus_trace_en = Bool()
    val dbg_en = Bool()
  }
  val fromDA = new LoadDA2WB
  val ld_da_addr = UInt(40.W)
  val ld_da_bkpta_data = Bool()
  val ld_da_bkptb_data = Bool()
  val ld_da_iid = UInt(7.W)
  val ld_da_inst_vfls = Bool()
  val ld_da_inst_vld = Bool()
  val ld_da_preg = UInt(7.W)
  val ld_da_vreg = UInt(6.W)
  val fromPad = new Bundle{
    val yy_icg_scan_en = Bool()
  }
  val fromRB = new Bundle{
    val bkpta_data = Bool()
    val bkptb_data = Bool()
    val bus_err = Bool()
    val bus_err_addr = UInt(40.W)
    val cmplt_req = Bool()
    val data = UInt(64.W)
    val data_iid = UInt(7.W)
    val data_req = Bool()
    val expt_gateclk = Bool()
    val expt_vld = Bool()
    val iid = UInt(7.W)
    val inst_vfls = Bool()
    val preg = UInt(7.W)
    val preg_sign_sel = UInt(4.W)
    val vreg = UInt(6.W)
    val vreg_sign_sel = UInt(2.W)
  }
  val fromRTU = new Bundle{
    val yy_xx_flush = Bool()
  }
  val vmb_ld_wb_data_req = Bool()
  val fromWmb = new Bundle{
    val data = UInt(64.W)
    val data_addr = UInt(40.W)
    val data_iid = UInt(7.W)
    val data_req = Bool()
    val inst_vfls = Bool()
    val preg = UInt(7.W)
    val preg_sign_sel = UInt(4.W)
    val vreg = UInt(6.W)
    val vreg_sign_sel = UInt(2.W)
  }
}

class LoadWBOutput extends Bundle{
  val ld_wb_data_vld = Bool()
  val ld_wb_inst_vld = Bool()
  val ld_wb_rb_cmplt_grnt = Bool()
  val ld_wb_rb_data_grnt = Bool()
  val ld_wb_wmb_data_grnt = Bool()
  val toHad = new Bundle{
    val ld_addr = UInt(40.W)
    val ld_data = UInt(64.W)
    val ld_iid = UInt(7.W)
    val ld_req = Bool()
    val ld_type = UInt(4.W)
  }
  val toIDU = new Bundle{
    val pipe3_fwd_vreg = UInt(7.W)
    val pipe3_fwd_vreg_vld = Bool()
    val pipe3_wb_preg = UInt(7.W)
    val pipe3_wb_preg_data = UInt(64.W)
    val pipe3_wb_preg_dup = Vec(5,UInt(7.W))
    val pipe3_wb_preg_expand = UInt(96.W)
    val pipe3_wb_preg_vld = Bool()
    val pipe3_wb_preg_vld_dup = Vec(5,Bool())
    val pipe3_wb_vreg_dup = Vec(4,UInt(7.W))
    val pipe3_wb_vreg_fr_data = UInt(64.W)
    val pipe3_wb_vreg_fr_expand = UInt(64.W)
    val pipe3_wb_vreg_fr_vld = Bool()
    val pipe3_wb_vreg_vld_dup = Vec(4,Bool())
    val pipe3_wb_vreg_vr_data   = Vec(2,UInt(64.W))
    val pipe3_wb_vreg_vr_expand = Vec(2,UInt(64.W))
    val pipe3_wb_vreg_vr_vld    = Vec(2,Bool())
  }
  val toRTU = new Bundle{
    val async_expt_addr = UInt(40.W)
    val async_expt_vld = Bool()
    val pipe3_abnormal = Bool()
    val pipe3_bkpta_data = Bool()
    val pipe3_bkptb_data = Bool()
    val pipe3_cmplt = Bool()
    val pipe3_expt_vec = UInt(5.W)
    val pipe3_expt_vld = Bool()
    val pipe3_flush = Bool()
    val pipe3_iid = UInt(7.W)
    val pipe3_mtval = UInt(40.W)
    val pipe3_no_spec_hit = Bool()
    val pipe3_no_spec_mispred = Bool()
    val pipe3_no_spec_miss = Bool()
    val pipe3_spec_fail = Bool()
    val pipe3_wb_preg_expand = UInt(96.W)
    val pipe3_wb_preg_vld = Bool()
    val pipe3_wb_vreg_expand = UInt(64.W)
    val pipe3_wb_vreg_fr_vld = Bool()
    val pipe3_wb_vreg_vr_vld = Bool()
  }
}

class LoadWBIO extends Bundle{
  val in  = Input(new LoadWBInput)
  val out = Output(new LoadWBOutput)
}

class LoadWB extends Module with LsuConfig{
  val io = IO(new LoadWBIO)

  //Reg
  val ld_wb_inst_vld = RegInit(false.B)

  val ld_wb_expt_vld        = RegInit(false.B)
  val ld_wb_iid             = RegInit(0.U(7.W))
  val ld_wb_spec_fail       = RegInit(false.B)
  val ld_wb_flush           = RegInit(false.B)
  val ld_wb_bkpta_data      = RegInit(false.B)
  val ld_wb_bkptb_data      = RegInit(false.B)
  val ld_wb_no_spec_miss    = RegInit(false.B)
  val ld_wb_no_spec_hit     = RegInit(false.B)
  val ld_wb_no_spec_mispred = RegInit(false.B)

  val ld_wb_expt_vec = RegInit(0.U(5.W))
  val ld_wb_mt_value = RegInit(0.U(PA_WIDTH.W))

  val ld_wb_data_vld        = RegInit(false.B)
  val ld_wb_preg_wb_vld     = RegInit(false.B)
  val ld_wb_preg_wb_vld_dup = RegInit(VecInit(Seq.fill(5)(false.B)))
  val ld_wb_vreg_wb_vld     = RegInit(false.B)
  val ld_wb_vreg_wb_vld_dup = RegInit(VecInit(Seq.fill(4)(false.B)))

  val ld_wb_bus_err = RegInit(false.B)
  val ld_wb_data_addr = RegInit(0.U(PA_WIDTH.W))
  val ld_wb_data_iid = RegInit(0.U(7.W))

  val ld_wb_data = RegInit(0.U(64.W))

  val ld_wb_data_preg = RegInit(0.U(7.W))
  val ld_wb_data_preg_dup = RegInit(VecInit(Seq.fill(5)(0.U(7.W))))
  val ld_wb_data_preg_expand = RegInit(0.U(96.W))
  val ld_wb_preg_sign_sel = RegInit(0.U(4.W))

  val ld_wb_data_vreg = RegInit(0.U(6.W))
  val ld_wb_data_vreg_dup = RegInit(VecInit(Seq.fill(4)(0.U(6.W))))
  val ld_wb_data_vreg_expand = RegInit(0.U(64.W))
  val ld_wb_vreg_sign_sel = RegInit(0.U(2.W))

  val wb_dbg_ld_req_ff = RegInit(false.B)

  val wb_dbg_ld_addr_ff = RegInit(0.U(PA_WIDTH.W))
  val wb_dbg_ld_data_ff = RegInit(0.U(64.W))
  val wb_dbg_ld_iid_ff  = RegInit(0.U(7.W))
  //Wire
  val ld_wb_pre_expt_vld = Wire(Bool())
  //==========================================================
  //                 arbitrate WB stage request
  //==========================================================
  val ld_wb_pre_mt_value   = io.in.fromDA.mt_value
  val ld_wb_pre_vstart_vld = false.B
  val ld_wb_pre_vsetvl     = false.B
  val ld_da_wb_vsetvl      = false.B
  //------------------complete part---------------------------
  //-----------grant signal---------------
  val ld_wb_da_cmplt_grnt = io.in.fromDA.cmplt_req

  val ld_wb_rb_cmplt_grnt = !io.in.fromDA.cmplt_req && io.in.fromRB.cmplt_req
  io.out.ld_wb_rb_cmplt_grnt := ld_wb_rb_cmplt_grnt

  //-----------signal select--------------
  val ld_wb_pre_inst_vld = io.in.fromDA.cmplt_req || io.in.fromRB.cmplt_req

  val ld_wb_pre_spec_fail = ld_wb_da_cmplt_grnt && io.in.fromDA.spec_fail

  val ld_wb_pre_bkpta_data = ld_wb_da_cmplt_grnt && io.in.ld_da_bkpta_data || ld_wb_rb_cmplt_grnt && io.in.fromRB.bkpta_data
  val ld_wb_pre_bkptb_data = ld_wb_da_cmplt_grnt && io.in.ld_da_bkptb_data || ld_wb_rb_cmplt_grnt && io.in.fromRB.bkptb_data

  val ld_wb_pre_flush = ld_wb_pre_spec_fail || ld_wb_pre_vstart_vld && !ld_wb_pre_expt_vld || ld_wb_pre_vsetvl

  ld_wb_pre_expt_vld := ld_wb_da_cmplt_grnt && io.in.fromDA.expt_vld || ld_wb_rb_cmplt_grnt && io.in.fromRB.expt_vld

  val ld_wb_pre_expt_gateclk = ld_wb_da_cmplt_grnt &&  (io.in.fromDA.expt_vld || ld_da_wb_vsetvl) ||
    ld_wb_rb_cmplt_grnt &&  io.in.fromRB.expt_gateclk

  val ld_wb_pre_expt_vec = Mux(ld_wb_da_cmplt_grnt, io.in.fromDA.expt_vec, 0.U(5.W)) | Mux(ld_wb_rb_cmplt_grnt, 5.U(5.W), 0.U(5.W))

  val ld_wb_pre_iid = Mux(ld_wb_da_cmplt_grnt, io.in.ld_da_iid, 0.U(7.W)) | Mux(ld_wb_rb_cmplt_grnt, io.in.fromRB.iid, 0.U(7.W))


  //------------------data part-------------------------------
  //-----------grant signal---------------
  val ld_wb_da_data_grnt = io.in.fromDA.data_req

  val ld_wb_wmb_data_grnt = !io.in.fromDA.data_req && io.in.fromWmb.data_req
  io.out.ld_wb_wmb_data_grnt := ld_wb_wmb_data_grnt

  val ld_wb_rb_data_grnt = !io.in.fromDA.data_req && !io.in.fromWmb.data_req && !io.in.vmb_ld_wb_data_req && io.in.fromRB.data_req
  io.out.ld_wb_rb_data_grnt := ld_wb_rb_data_grnt

  //-----------signal select--------------
  val ld_wb_pre_data_vld = io.in.fromDA.data_req || io.in.fromWmb.data_req || io.in.vmb_ld_wb_data_req || io.in.fromRB.data_req

  val ld_wb_pre_bus_err = ld_wb_rb_data_grnt &&  io.in.fromRB.bus_err

  val ld_wb_pre_data_addr = Mux(ld_wb_da_data_grnt, io.in.ld_da_addr, 0.U(PA_WIDTH.W)) |
    Mux(ld_wb_wmb_data_grnt, io.in.fromWmb.data_addr,  0.U(PA_WIDTH.W)) |
    Mux(ld_wb_rb_data_grnt, io.in.fromRB.bus_err_addr, 0.U(PA_WIDTH.W))

  //for had debug
  val ld_wb_pre_data_iid = Mux(ld_wb_da_data_grnt, io.in.ld_da_iid, 0.U(7.W)) |
    Mux(ld_wb_wmb_data_grnt, io.in.fromWmb.data_iid, 0.U(7.W)) |
    Mux(ld_wb_rb_data_grnt, io.in.fromRB.data_iid, 0.U(7.W))

  val ld_wb_pre_preg = Mux(ld_wb_da_data_grnt, io.in.ld_da_preg, 0.U(7.W)) |
    Mux(ld_wb_wmb_data_grnt, io.in.fromWmb.preg, 0.U(7.W)) |
    Mux(ld_wb_rb_data_grnt, io.in.fromRB.preg, 0.U(7.W))

  val ld_wb_pre_vreg = Mux(ld_wb_da_data_grnt, io.in.ld_da_vreg, 0.U(6.W)) |
    Mux(ld_wb_wmb_data_grnt, io.in.fromWmb.vreg, 0.U(6.W)) |
    Mux(ld_wb_rb_data_grnt, io.in.fromRB.vreg, 0.U(6.W))

  val ld_wb_pre_preg_expand = UIntToOH(ld_wb_pre_preg, 96)
  val ld_wb_pre_vreg_expand = UIntToOH(ld_wb_pre_vreg, 64)
  //preg expand to 96 bits
  //do 3 times for timing //TODO: why?

  val ld_wb_pre_data_no_da = Mux(io.in.fromWmb.data_req, io.in.fromWmb.data, io.in.fromRB.data)

  val ld_wb_pre_data = Mux(io.in.fromDA.data_req, io.in.fromDA.data, ld_wb_pre_data_no_da)

  val ld_wb_pre_inst_vfls = ld_wb_da_data_grnt  & io.in.ld_da_inst_vfls |
    ld_wb_wmb_data_grnt  & io.in.fromWmb.inst_vfls |
    ld_wb_rb_data_grnt  & io.in.fromRB.inst_vfls

  val ld_wb_pre_preg_wb_vld  = ld_wb_pre_data_vld && !ld_wb_pre_inst_vfls
  val ld_wb_pre_vreg_wb_vld  = ld_wb_pre_data_vld && ld_wb_pre_inst_vfls

  //because the timing in ld_da is not enough, so some of the sign extend
  //precedure is done in wb stage
  val ld_wb_pre_preg_sign_sel = Mux(ld_wb_da_data_grnt, io.in.fromDA.preg_sign_sel, 0.U(4.W)) |
    Mux(ld_wb_wmb_data_grnt, io.in.fromWmb.preg_sign_sel, 0.U(4.W)) |
    Mux(ld_wb_rb_data_grnt, io.in.fromRB.preg_sign_sel, 0.U(4.W))

  val ld_wb_pre_vreg_sign_sel = Mux(ld_wb_da_data_grnt, io.in.fromDA.vreg_sign_sel, 0.U(2.W)) |
    Mux(ld_wb_wmb_data_grnt, io.in.fromWmb.vreg_sign_sel, 0.U(2.W)) |
    Mux(ld_wb_rb_data_grnt, io.in.fromRB.vreg_sign_sel, 0.U(2.W))

  val ld_wb_data_pre_vmb_merge_vld = false.B

  val ld_wb_pre_no_spec_miss    = ld_wb_da_cmplt_grnt &&  io.in.fromDA.no_spec_miss
  val ld_wb_pre_no_spec_hit     = ld_wb_da_cmplt_grnt &&  io.in.fromDA.no_spec_hit
  val ld_wb_pre_no_spec_mispred = ld_wb_da_cmplt_grnt &&  io.in.fromDA.no_spec_mispred

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //------------------cmplt part gateclk----------------------
  val ld_wb_cmplt_clk_en = io.in.ld_da_inst_vld || io.in.fromRB.cmplt_req

  val ld_wb_expt_clk_en  = ld_wb_pre_expt_gateclk

  //------------------data part gateclk-----------------------
  val ld_wb_data_clk_en = ld_wb_pre_data_vld

  val ld_wb_preg_clk_en = io.in.fromDA.data_req_gateclk_en && !io.in.ld_da_inst_vfls || io.in.fromRB.data_req && !io.in.fromRB.inst_vfls || io.in.fromWmb.data_req && !io.in.fromWmb.inst_vfls

  val ld_wb_vreg_clk_en = io.in.fromDA.data_req_gateclk_en && io.in.ld_da_inst_vfls || io.in.fromRB.data_req && io.in.fromRB.inst_vfls || io.in.fromWmb.data_req && io.in.fromWmb.inst_vfls

  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------complete part---------------------------
  //+----------+----------+
  //| inst_vld | expt_vld |
  //+----------+----------+
  when(io.in.fromRTU.yy_xx_flush){
    ld_wb_inst_vld := false.B
  }.elsewhen(ld_wb_pre_inst_vld){
    ld_wb_inst_vld := true.B
  }.otherwise{
    ld_wb_inst_vld := false.B
  }
  io.out.ld_wb_inst_vld := ld_wb_inst_vld

  //+-----+---------------+-------+-------+
  //| iid | rar_spec_fail | bkpta | bkptb |
  //+-----+---------------+-------+-------+
  when(ld_wb_pre_inst_vld){
    ld_wb_expt_vld        :=  ld_wb_pre_expt_vld
    ld_wb_iid             :=  ld_wb_pre_iid
    ld_wb_spec_fail       :=  ld_wb_pre_spec_fail
    ld_wb_flush           :=  ld_wb_pre_flush
    ld_wb_bkpta_data      :=  ld_wb_pre_bkpta_data
    ld_wb_bkptb_data      :=  ld_wb_pre_bkptb_data
    ld_wb_no_spec_miss    :=  ld_wb_pre_no_spec_miss
    ld_wb_no_spec_hit     :=  ld_wb_pre_no_spec_hit
    ld_wb_no_spec_mispred :=  ld_wb_pre_no_spec_mispred
  }

  //+----------+---------+-----------+
  //| expt_vec | bad_vpn | dmmu_expt |
  //+----------+---------+-----------+
  when(ld_wb_pre_expt_vld || ld_wb_pre_vsetvl){
    ld_wb_expt_vec :=  ld_wb_pre_expt_vec
    ld_wb_mt_value :=  ld_wb_pre_mt_value
  }

  //------------------data part-------------------------------
  //+----------+---------+
  //| data_vld | bus_err |
  //+----------+---------+
  when(io.in.fromRTU.yy_xx_flush){
    ld_wb_data_vld        := false.B
    ld_wb_preg_wb_vld     := false.B
    ld_wb_preg_wb_vld_dup := WireInit(VecInit(Seq.fill(5)(false.B)))
    ld_wb_vreg_wb_vld     := false.B
    ld_wb_vreg_wb_vld_dup := WireInit(VecInit(Seq.fill(4)(false.B)))
  }.elsewhen(ld_wb_pre_data_vld){
    ld_wb_data_vld        := true.B
    ld_wb_preg_wb_vld     := ld_wb_pre_preg_wb_vld
    ld_wb_preg_wb_vld_dup := WireInit(VecInit(Seq.fill(5)(ld_wb_pre_preg_wb_vld)))
    ld_wb_vreg_wb_vld     := ld_wb_pre_vreg_wb_vld && !ld_wb_data_pre_vmb_merge_vld
    ld_wb_vreg_wb_vld_dup := WireInit(VecInit(Seq.fill(4)(ld_wb_pre_vreg_wb_vld && !ld_wb_data_pre_vmb_merge_vld)))
  }.otherwise{
    ld_wb_data_vld        := false.B
    ld_wb_preg_wb_vld     := false.B
    ld_wb_preg_wb_vld_dup := WireInit(VecInit(Seq.fill(5)(false.B)))
    ld_wb_vreg_wb_vld     := false.B
    ld_wb_vreg_wb_vld_dup := WireInit(VecInit(Seq.fill(4)(false.B)))
  }
  io.out.ld_wb_data_vld := ld_wb_data_vld

  when(io.in.fromRTU.yy_xx_flush){
    ld_wb_bus_err := false.B
  }.elsewhen(ld_wb_pre_data_vld){
    ld_wb_bus_err := ld_wb_pre_bus_err
  }.otherwise{
    ld_wb_bus_err := false.B
  }

  when(ld_wb_pre_data_vld){
    ld_wb_data_addr :=  ld_wb_pre_data_addr
    ld_wb_data_iid  :=  ld_wb_pre_data_iid
  }

  //+------+----------+------+
  //| data | sign_sel | sign |
  //+------+----------+------+
  when(ld_wb_pre_data_vld){
    ld_wb_data := ld_wb_pre_data
  }

  //+------+
  //| preg |
  //+------+
  when(ld_wb_pre_preg_wb_vld){
    ld_wb_data_preg        :=  ld_wb_pre_preg
    ld_wb_data_preg_dup    :=  WireInit(VecInit(Seq.fill(5)(ld_wb_pre_preg)))
    ld_wb_data_preg_expand :=  ld_wb_pre_preg_expand
    ld_wb_preg_sign_sel    :=  ld_wb_pre_preg_sign_sel
  }

  //+------+
  //| vreg |
  //+------+
  when(ld_wb_pre_vreg_wb_vld){
    ld_wb_data_vreg        :=  ld_wb_pre_vreg
    ld_wb_data_vreg_dup    :=  WireInit(VecInit(Seq.fill(4)(ld_wb_pre_vreg)))
    ld_wb_data_vreg_expand :=  ld_wb_pre_vreg_expand
    ld_wb_vreg_sign_sel    :=  ld_wb_pre_vreg_sign_sel
  }

  //==========================================================
  //            Data settle and Sign extend
  //==========================================================
  //sign extend
  val ld_wb_data_sign = Wire(Vec(4, UInt(64.W)))
  ld_wb_data_sign(0) := ld_wb_data
  ld_wb_data_sign(1) := sext(64, ld_wb_data(7,0))
  ld_wb_data_sign(2) := sext(64, ld_wb_data(15,0))
  ld_wb_data_sign(3) := sext(64, ld_wb_data(31,0))

  val ld_wb_preg_data_sign_extend = Mux(ld_wb_preg_sign_sel(0), ld_wb_data_sign(0), 0.U(64.W)) |
    Mux(ld_wb_preg_sign_sel(1), ld_wb_data_sign(1), 0.U(64.W)) |
    Mux(ld_wb_preg_sign_sel(2), ld_wb_data_sign(2), 0.U(64.W)) |
    Mux(ld_wb_preg_sign_sel(3), ld_wb_data_sign(3), 0.U(64.W))

  val ld_wb_vreg_data_sign_extend = MuxLookup(ld_wb_vreg_sign_sel, ld_wb_data, Seq(
    "b01".U -> Cat(VecInit(Seq.fill(48)(true.B)).asUInt, ld_wb_data(15,0)),
    "b10".U -> Cat(VecInit(Seq.fill(32)(true.B)).asUInt, ld_wb_data(31,0))
  ))

  //==========================================================
  //                 Generate interface to rtu
  //==========================================================
  //------------------complete part---------------------------
  io.out.toRTU.pipe3_cmplt           := ld_wb_inst_vld
  io.out.toRTU.pipe3_iid             := ld_wb_iid
  io.out.toRTU.pipe3_expt_vld        := ld_wb_expt_vld
  io.out.toRTU.pipe3_expt_vec        := ld_wb_expt_vec
  io.out.toRTU.pipe3_mtval           := ld_wb_mt_value
  io.out.toRTU.pipe3_spec_fail       := ld_wb_spec_fail
  io.out.toRTU.pipe3_bkpta_data      := ld_wb_bkpta_data
  io.out.toRTU.pipe3_bkptb_data      := ld_wb_bkptb_data
  io.out.toRTU.pipe3_flush           := ld_wb_flush
  io.out.toRTU.pipe3_abnormal        := ld_wb_expt_vld || ld_wb_flush

  io.out.toRTU.pipe3_no_spec_miss    := ld_wb_no_spec_miss
  io.out.toRTU.pipe3_no_spec_hit     := ld_wb_no_spec_hit
  io.out.toRTU.pipe3_no_spec_mispred := ld_wb_no_spec_mispred

  //------------------data part-------------------------------
  io.out.toRTU.pipe3_wb_preg_vld    := ld_wb_preg_wb_vld
  io.out.toRTU.pipe3_wb_preg_expand := ld_wb_data_preg_expand

  io.out.toRTU.async_expt_vld       := ld_wb_data_vld && ld_wb_bus_err
  io.out.toRTU.async_expt_addr      := Mux(ld_wb_data_vld  &&  ld_wb_bus_err, ld_wb_data_addr, 0.U(PA_WIDTH.W))

  io.out.toIDU.pipe3_wb_preg_vld     := ld_wb_preg_wb_vld
  io.out.toIDU.pipe3_wb_preg_vld_dup := ld_wb_preg_wb_vld_dup
  io.out.toIDU.pipe3_wb_preg         := ld_wb_data_preg
  io.out.toIDU.pipe3_wb_preg_dup     := ld_wb_data_preg_dup
  io.out.toIDU.pipe3_wb_preg_expand  := ld_wb_data_preg_expand
  io.out.toIDU.pipe3_wb_preg_data    := ld_wb_preg_data_sign_extend

  //------------------for vector------------------------
  io.out.toRTU.pipe3_wb_vreg_fr_vld          := ld_wb_vreg_wb_vld
  io.out.toRTU.pipe3_wb_vreg_vr_vld          := false.B
  io.out.toRTU.pipe3_wb_vreg_expand          := ld_wb_data_vreg_expand

  io.out.toIDU.pipe3_wb_vreg_fr_vld          := ld_wb_vreg_wb_vld
  io.out.toIDU.pipe3_wb_vreg_fr_expand       := ld_wb_data_vreg_expand
  io.out.toIDU.pipe3_wb_vreg_fr_data         := ld_wb_vreg_data_sign_extend

  for(i <- 0 until 2){
    io.out.toIDU.pipe3_wb_vreg_vr_vld(i)     := false.B
    io.out.toIDU.pipe3_wb_vreg_vr_expand(i)  := 0.U(64.W)
    io.out.toIDU.pipe3_wb_vreg_vr_data(i)    := 0.U(64.W)
  }

  io.out.toIDU.pipe3_fwd_vreg_vld  := ld_wb_vreg_wb_vld
  io.out.toIDU.pipe3_fwd_vreg      := Cat(0.U(1.W),ld_wb_data_vreg)

  io.out.toIDU.pipe3_wb_vreg_vld_dup := ld_wb_vreg_wb_vld_dup

  io.out.toIDU.pipe3_wb_vreg_dup     := WireInit(VecInit(Seq.fill(4)(Cat(0.U(1.W), ld_wb_data_vreg_dup(0)))))

  //==========================================================
  //                 Generate interface to rtu
  //==========================================================
  val wb_dbg_ld_req = ld_wb_preg_wb_vld

  val wb_dbg_clk_en  = io.in.fromHad.dbg_en &&  (wb_dbg_ld_req || wb_dbg_ld_req_ff)

  when(wb_dbg_ld_req){
    wb_dbg_ld_req_ff := true.B
  }.otherwise{
    wb_dbg_ld_req_ff := false.B
  }

  when(wb_dbg_ld_req){
    wb_dbg_ld_addr_ff :=  ld_wb_data_addr
    wb_dbg_ld_data_ff :=  ld_wb_preg_data_sign_extend
    wb_dbg_ld_iid_ff  :=  ld_wb_data_iid
  }

  val wb_dbg_ar_req_ff =  false.B
  val wb_dbg_ar_addr   =  0.U(PA_WIDTH.W)
  val wb_dbg_ar_id     =  0.U(8.W)
  val wb_dbg_ar_len    =  0.U(8.W)
  val wb_dbg_ar_size   =  0.U(3.W)
  val wb_dbg_ar_burst  =  0.U(2.W)
  val wb_dbg_ar_lock   =  false.B
  val wb_dbg_ar_cache  =  0.U(4.W)
  val wb_dbg_ar_prot   =  0.U(3.W)
  val wb_dbg_ar_snoop  =  0.U(4.W)
  val wb_dbg_ar_domain =  0.U(2.W)
  val wb_dbg_ar_bar    =  0.U(2.W)

  io.out.toHad.ld_req := Mux(io.in.fromHad.bus_trace_en, wb_dbg_ar_req_ff, wb_dbg_ld_req_ff)

  io.out.toHad.ld_addr := Mux(io.in.fromHad.bus_trace_en, wb_dbg_ar_addr, wb_dbg_ld_addr_ff)

  io.out.toHad.ld_data := Mux(io.in.fromHad.bus_trace_en, 0.U(64.W), wb_dbg_ld_data_ff)

  io.out.toHad.ld_iid := Mux(io.in.fromHad.bus_trace_en, 0.U(7.W), wb_dbg_ld_iid_ff)

  io.out.toHad.ld_type := Mux(io.in.fromHad.bus_trace_en, 4.U(4.W), 2.U(4.W))

}
