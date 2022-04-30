package Core.LSU.Rb
import Core.LsuConfig
import chisel3._
import chisel3.util._

class RbEntryInstInfo extends Bundle with LsuConfig{
  val mcic_req          = Bool()
  val addr              = UInt(PA_WIDTH.W)
  val bytes_vld         = UInt(16.W)
  val iid               = UInt(7.W)
  val fence_mode        = UInt(4.W)
  val sign_extend       = Bool()
  val boundary          = Bool()
  val preg              = UInt(7.W)
  val page_ca           = Bool()
  val page_so           = Bool()
  val page_sec          = Bool()
  val page_buf          = Bool()
  val page_share        = Bool()
  val sync              = Bool()
  val fence             = Bool()
  val atomic            = Bool()
  val ldamo             = Bool()
  val dcache_hit        = Bool()
  val st                = Bool()
  val inst_size         = UInt(3.W)
  val create_lfb        = Bool()
  val bkpta_data        = Bool()
  val bkptb_data        = Bool()
  val vreg              = UInt(6.W)
  val inst_vfls         = Bool()
  val vreg_sign_sel     = Bool()
  val priv_mode         = UInt(2.W)
}

class RbEntryIn extends Bundle{
  val create_ptr0 = Bool()
  val biu_id_gateclk_en = Bool()
  val biu_pe_req_grnt = Bool()
  val ld_create_dp_vld = Bool()
  val ld_create_gateclk_en = Bool()
  val ld_create_vld = Bool()
  val next_nc_bypass = Bool()
  val next_so_bypass = Bool()
  val read_req_grnt = Bool()
  val st_create_dp_vld = Bool()
  val st_create_gateclk_en = Bool()
  val st_create_vld = Bool()
  val wb_cmplt_grnt = Bool()
  val wb_data_grnt = Bool()
}

class RbEntryInput extends Bundle with LsuConfig{
  val fromBiu = new Bundle{
    val b_id = UInt(5.W)
    val b_vld = Bool()
    val r_data_mask = UInt(128.W)
    val r_id = UInt(5.W)
    val r_vld = Bool()
  }
  val fromCp0 = new RbFromCp0
  val fromLoadDA = new RbFromLoadDA

  val lm_already_snoop = Bool()
  val lsu_has_fence = Bool()
  val lsu_special_clk = Bool()
  val pad_yy_icg_scan_en = Bool()
  val pfu_biu_req_addr = UInt(PA_WIDTH.W)
  val rb_biu_ar_id = UInt(5.W)

  val RbEntry = new RbEntryIn

  val rb_fence_ld = Bool()
  val rb_ld_biu_pe_req_grnt = Bool()
  val rb_r_resp_err = Bool()
  val rb_r_resp_okay = Bool()
  val rb_ready_all_req_biu_success = Bool()
  val rb_ready_ld_req_biu_success = Bool()

  val fromRTU = new RbFromRTU
  val fromSQ = new Bundle{
    val pop_addr = UInt(PA_WIDTH.W)
    val pop_page_ca = Bool()
    val pop_page_so = Bool()
  }
  val fromStoreDA = new RbFromStoreDA
  val fromWmb = new RbFromWmb
}

class RbEntryOutput extends Bundle with LsuConfig{
  val addr = UInt(PA_WIDTH.W)
  val atomic_next_resp = Bool()
  val atomic = Bool()
  val biu_pe_req_gateclk_en = Bool()
  val biu_pe_req = Bool()
  val biu_req = Bool()
  val bkpta_data = Bool()
  val bkptb_data = Bool()
  val boundary_wakeup = Bool()
  val bus_err = Bool()
  val cmit_data_vld = Bool()
  val create_lfb = Bool()
  val data = UInt(64.W)
  val depd = Bool()
  val discard_vld = Bool()
  val fence_ld_vld = Bool()
  val fence = Bool()
  val flush_clear = Bool()
  val iid = UInt(7.W)
  val inst_size = UInt(3.W)
  val inst_vfls = Bool()
  val ld_da_hit_idx = Bool()
  val ldamo = Bool()
  val mcic_req = Bool()
  val merge_fail = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val pfu_biu_req_hit_idx = Bool()
  val preg = UInt(7.W)
  val priv_mode = UInt(2.W)
  val rot_sel = UInt(8.W)
  val sign_extend = Bool()
  val sq_pop_hit_idx = Bool()
  val st_da_hit_idx = Bool()
  val st = Bool()
  val state = UInt(4.W)
  val sync_fence = Bool()
  val sync = Bool()
  val vld = Bool()
  val vreg_sign_sel = Bool()
  val vreg = UInt(6.W)
  val wb_cmplt_req = Bool()
  val wb_data_pre_sel = Bool()
  val wb_data_req = Bool()
  val wmb_ce_hit_idx = Bool()
}

class RbEntryIO extends Bundle{
  val in = Input(new RbEntryInput)
  val out = Output(new RbEntryOutput)
}

class RbEntry extends Module with LsuConfig {
  val io = IO(new RbEntryIO)

  //FSM
  //the state machine is devided to 2 part:
  //before request biu: state[2] = 0
  //after request biu:  state[2] = 1
  def IDLE        = "b0000".U
  def WAIT_RDY    = "b1000".U
  def REQ_BIU     = "b1001".U
  def WAIT_RESP   = "b1100".U
  def REQ_WB      = "b1101".U
  def WAIT_MERGE  = "b1110".U

  //Reg
  val rb_entry_state = RegInit(IDLE)
  val rb_entry_cmit = RegInit(false.B)
  val inst_info = RegInit(0.U.asTypeOf(new RbEntryInstInfo))
  val rb_entry_rot_sel = RegInit(0.U(8.W))
  val rb_entry_secd = RegInit(false.B)
  val rb_entry_depd = RegInit(false.B)
  val rb_entry_boundary_depd = RegInit(false.B)
  val rb_entry_dest_vld = RegInit(false.B)
  val rb_entry_data = RegInit(0.U(64.W))
  val rb_entry_wb_cmplt_success = RegInit(false.B)
  val rb_entry_wb_data_success = RegInit(false.B)
  val rb_entry_biu_id = RegInit(0.U(5.W))
  val rb_entry_biu_r_resp = RegInit(false.B)
  val rb_entry_biu_b_resp = RegInit(false.B)
  val rb_entry_bus_err = RegInit(false.B)
  val rb_entry_biu_pe_req_success = RegInit(false.B)

  //Wire
  val rb_entry_vld = Wire(Bool())
  val rb_entry_st_create_gateclk_en = Wire(Bool())
  val rb_entry_ld_create_gateclk_en = Wire(Bool())
  val rb_entry_ld_merge_gateclk_en = Wire(Bool())
  val rb_entry_data_bypass_vld = Wire(Bool())
  val rb_entry_biu_id_gateclk_en = Wire(Bool())

  val rb_entry_next_state = Wire(UInt(4.W))

  val rb_entry_ld_create_dp_vld = Wire(Bool())
  val rb_entry_st_create_dp_vld = Wire(Bool())
  val rb_entry_cmit_set = Wire(Bool())
  val rb_entry_ld_merge_dp_vld = Wire(Bool())
  val rb_entry_ld_merge_vld = Wire(Bool())
  val rb_entry_create_dp_vld = Wire(Bool())
  val rb_entry_discard_vld = Wire(Bool())
  val rb_entry_boundary_wakeup = Wire(Bool())
  val rb_entry_boundary_depd_set = Wire(Bool())
  val rb_entry_ld_merge_expt_vld = Wire(Bool())
  val rb_entry_flush_clear = Wire(Bool())

  val rb_entry_merge_data = Wire(UInt(64.W))
  val rb_entry_biu_data_update = Wire(UInt(64.W))

  val rb_entry_wb_cmplt_grnt = Wire(Bool())
  val rb_entry_wb_data_grnt = Wire(Bool())
  val rb_entry_fof_bus_err_expt = Wire(Bool())
  val rb_entry_read_req_grnt = Wire(Bool())
  val rb_entry_biu_r_resp_set = Wire(Bool())
  val rb_entry_biu_b_resp_set = Wire(Bool())
  val rb_entry_bus_err_set = Wire(Bool())
  val rb_entry_fof_not_first = Wire(Bool())
  val rb_entry_create_vld = Wire(Bool())
  val rb_entry_biu_pe_req_grnt = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //-----------entry gateclk--------------
  //normal gateclk ,open when create || entry_vld
  val rb_entry_clk_en = rb_entry_vld || rb_entry_st_create_gateclk_en || rb_entry_ld_create_gateclk_en

  //-----------create merge gateclk-------
  val rb_entry_create_up_clk_en  = rb_entry_ld_create_gateclk_en || rb_entry_st_create_gateclk_en || rb_entry_ld_merge_gateclk_en

  //-----------data gateclk---------------
  //data gateclk, delete if the timing is bad
  val rb_entry_data_clk_en = rb_entry_ld_create_gateclk_en || rb_entry_ld_merge_gateclk_en || rb_entry_data_bypass_vld

  //----------biu_id gateclk--------------
  val rb_entry_biu_id_clk_en = rb_entry_biu_id_gateclk_en

  //==========================================================
  //                 Register
  //==========================================================
  //+-------+
  //| state |
  //+-------+
  rb_entry_state := rb_entry_next_state

  rb_entry_vld := rb_entry_state(3)
  val rb_entry_biu_req_success = rb_entry_state(2)

  //+------+
  //| cmit |
  //+------+
  when(rb_entry_ld_create_dp_vld){
    rb_entry_cmit := io.in.fromLoadDA.rb_cmit
  }.elsewhen(rb_entry_st_create_dp_vld){
    rb_entry_cmit := io.in.fromStoreDA.rb_cmit
  }.elsewhen(rb_entry_cmit_set){
    rb_entry_cmit := true.B
  }

  //+-------------------------+
  //| instruction information |
  //+-------------------------+
  when(rb_entry_ld_merge_dp_vld){
    inst_info.mcic_req      := false.B
    inst_info.addr          := io.in.fromLoadDA.addr
    inst_info.bytes_vld     := io.in.fromLoadDA.bytes_vld
    inst_info.iid           := io.in.fromLoadDA.iid
    inst_info.fence_mode    := 0.U(4.W)
    inst_info.sign_extend   := io.in.fromLoadDA.sign_extend
    inst_info.boundary      := io.in.fromLoadDA.boundary_after_mask
    inst_info.preg          := io.in.fromLoadDA.preg
    inst_info.page_ca       := io.in.fromLoadDA.page_ca
    inst_info.page_so       := io.in.fromLoadDA.page_so
    inst_info.page_sec      := io.in.fromLoadDA.page_sec
    inst_info.page_buf      := io.in.fromLoadDA.page_buf
    inst_info.page_share    := io.in.fromLoadDA.page_share
    inst_info.sync          := false.B
    inst_info.fence         := false.B
    inst_info.atomic        := false.B
    inst_info.ldamo         := false.B
    inst_info.dcache_hit    := io.in.fromLoadDA.dcache_hit
    inst_info.st            := false.B
    inst_info.inst_size     := io.in.fromLoadDA.inst_size
    inst_info.create_lfb    := io.in.fromLoadDA.rb_create_lfb
    inst_info.bkpta_data    := io.in.fromLoadDA.bkpta_data
    inst_info.bkptb_data    := io.in.fromLoadDA.bkptb_data
    inst_info.vreg          := io.in.fromLoadDA.vreg
    inst_info.inst_vfls     := io.in.fromLoadDA.inst_vfls
    inst_info.vreg_sign_sel := io.in.fromLoadDA.vreg_sign_sel
    inst_info.priv_mode     := io.in.fromCp0.yy_priv_mode
  }.elsewhen(rb_entry_ld_create_dp_vld){
    inst_info.mcic_req      := io.in.fromLoadDA.mcic_borrow_mmu
    inst_info.addr          := io.in.fromLoadDA.addr
    inst_info.bytes_vld     := io.in.fromLoadDA.bytes_vld
    inst_info.iid           := io.in.fromLoadDA.iid
    inst_info.fence_mode    := 0.U(4.W)
    inst_info.sign_extend   := io.in.fromLoadDA.sign_extend
    inst_info.boundary      := io.in.fromLoadDA.boundary_after_mask
    inst_info.preg          := io.in.fromLoadDA.preg
    inst_info.page_ca       := io.in.fromLoadDA.page_ca
    inst_info.page_so       := io.in.fromLoadDA.page_so
    inst_info.page_sec      := io.in.fromLoadDA.page_sec
    inst_info.page_buf      := io.in.fromLoadDA.page_buf
    inst_info.page_share    := io.in.fromLoadDA.page_share
    inst_info.sync          := false.B
    inst_info.fence         := false.B
    inst_info.atomic        := io.in.fromLoadDA.rb_atomic
    inst_info.ldamo         := io.in.fromLoadDA.rb_ldamo
    inst_info.dcache_hit    := io.in.fromLoadDA.dcache_hit
    inst_info.st            := false.B
    inst_info.inst_size     := io.in.fromLoadDA.inst_size
    inst_info.create_lfb    := io.in.fromLoadDA.rb_create_lfb
    inst_info.bkpta_data    := io.in.fromLoadDA.bkpta_data
    inst_info.bkptb_data    := io.in.fromLoadDA.bkptb_data
    inst_info.vreg          := io.in.fromLoadDA.vreg
    inst_info.inst_vfls     := io.in.fromLoadDA.inst_vfls
    inst_info.vreg_sign_sel := io.in.fromLoadDA.vreg_sign_sel
    inst_info.priv_mode     := io.in.fromCp0.yy_priv_mode
  }.elsewhen(rb_entry_st_create_dp_vld){
    inst_info.mcic_req      := false.B
    inst_info.addr          := io.in.fromStoreDA.addr
    inst_info.bytes_vld     := 0.U(16.W)
    inst_info.iid           := io.in.fromStoreDA.iid
    inst_info.fence_mode    := io.in.fromStoreDA.fence_mode
    inst_info.sign_extend   := false.B
    inst_info.boundary      := false.B
    inst_info.preg          := 0.U(7.W)
    inst_info.page_ca       := io.in.fromStoreDA.page_ca
    inst_info.page_so       := io.in.fromStoreDA.page_so
    inst_info.page_sec      := io.in.fromStoreDA.page_sec
    inst_info.page_buf      := io.in.fromStoreDA.page_buf
    inst_info.page_share    := io.in.fromStoreDA.page_share
    inst_info.sync          := io.in.fromStoreDA.sync_inst
    inst_info.fence         := io.in.fromStoreDA.sync_fence
    inst_info.atomic        := false.B
    inst_info.ldamo         := false.B
    inst_info.dcache_hit    := io.in.fromStoreDA.dcache_hit
    inst_info.st            := false.B
    inst_info.inst_size     := io.in.fromStoreDA.inst_size
    inst_info.create_lfb    := io.in.fromStoreDA.rb_create_lfb
    inst_info.bkpta_data    := false.B
    inst_info.bkptb_data    := false.B
    inst_info.vreg          := 0.U(6.W)
    inst_info.inst_vfls     := false.B
    inst_info.vreg_sign_sel := false.B
    inst_info.priv_mode     := io.in.fromCp0.yy_priv_mode
  }

  //+---------+
  //| rot_sel |
  //+---------+
  //used for rot sel
  when(rb_entry_ld_create_dp_vld){
    rb_entry_rot_sel := io.in.fromLoadDA.data_rot_sel
  }

  //+------+
  //| secd |
  //+------+
  //secd must be accurate, so it use set signal
  when(rb_entry_ld_merge_vld){
    rb_entry_secd := true.B
  }.elsewhen(rb_entry_ld_create_dp_vld ||  rb_entry_st_create_dp_vld){
    rb_entry_secd := false.B
  }

  //+------+
  //| depd |
  //+------+
  when(rb_entry_ld_merge_dp_vld ||  rb_entry_create_dp_vld){
    rb_entry_depd := false.B
  }.elsewhen(rb_entry_discard_vld){
    rb_entry_depd := true.B
  }

  //+---------------+
  //| boundary_depd |
  //+---------------+
  when(rb_entry_create_dp_vld  ||  rb_entry_boundary_wakeup){
    rb_entry_boundary_depd := false.B
  }.elsewhen(rb_entry_boundary_depd_set){
    rb_entry_boundary_depd := true.B
  }

  //+----------+
  //| dest_vld |
  //+----------+
  when(rb_entry_ld_create_dp_vld){
    rb_entry_dest_vld := io.in.fromLoadDA.rb_dest_vld
  }.elsewhen(rb_entry_ld_merge_expt_vld || rb_entry_st_create_dp_vld || rb_entry_flush_clear){
    rb_entry_dest_vld := false.B
  }

  //+------+
  //| data |
  //+------+
  val rb_entry_data_merge_vld = rb_entry_ld_merge_dp_vld &&  io.in.fromLoadDA.rb_data_vld || rb_entry_data_bypass_vld && rb_entry_secd

  when(rb_entry_data_merge_vld){
    rb_entry_data := rb_entry_merge_data
  }.elsewhen(rb_entry_data_bypass_vld){
    rb_entry_data := rb_entry_biu_data_update
  }.elsewhen(rb_entry_ld_create_dp_vld && io.in.fromLoadDA.rb_data_vld){
    rb_entry_data := io.in.fromLoadDA.data_ori
  }

  //+------------------+
  //| wb_cmplt_success |
  //+------------------+
  when(rb_entry_ld_create_dp_vld){
    rb_entry_wb_cmplt_success := io.in.fromLoadDA.rb_cmplt_success
  }.elsewhen(rb_entry_st_create_dp_vld){
    rb_entry_wb_cmplt_success := true.B
  }.elsewhen(rb_entry_ld_merge_vld){
    rb_entry_wb_cmplt_success := io.in.fromLoadDA.rb_cmplt_success
  }.elsewhen(rb_entry_wb_cmplt_grnt){
    rb_entry_wb_cmplt_success := true.B
  }

  //+-----------------+
  //| wb_data_success |
  //+-----------------+
  when(rb_entry_ld_create_dp_vld){
    rb_entry_wb_data_success := false.B
  }.elsewhen(rb_entry_st_create_dp_vld){
    rb_entry_wb_data_success := true.B
  }.elsewhen(rb_entry_wb_data_grnt){
    rb_entry_wb_data_success := true.B
  }.elsewhen(rb_entry_fof_bus_err_expt){
    rb_entry_wb_data_success := true.B
  }

  //+--------+
  //| biu_id |
  //+--------+
  when(rb_entry_read_req_grnt){
    rb_entry_biu_id := io.in.rb_biu_ar_id
  }

  //+------------+
  //| biu_r_resp |
  //+------------+
  when(rb_entry_create_dp_vld){
    rb_entry_biu_r_resp := false.B
  }.elsewhen(rb_entry_biu_r_resp_set){
    rb_entry_biu_r_resp := true.B
  }

  //+------------+
  //| biu_b_resp |
  //+------------+
  when(rb_entry_create_dp_vld){
    rb_entry_biu_b_resp := false.B
  }.elsewhen(rb_entry_biu_b_resp_set){
    rb_entry_biu_b_resp := true.B
  }

  //+---------+
  //| bus_err |
  //+---------+
  //ecc err will not carry bus err expt
  when(rb_entry_create_dp_vld){
    rb_entry_bus_err := false.B
  }.elsewhen(rb_entry_bus_err_set && !rb_entry_fof_not_first){
    rb_entry_bus_err := true.B
  }

  //+-----------------------+
  //| biu_pop_entry_success |
  //+-----------------------+
  //this signal represents request biu_pop_entry successfully
  when(rb_entry_ld_create_dp_vld && io.in.rb_ld_biu_pe_req_grnt){
    rb_entry_biu_pe_req_success := true.B
  }.elsewhen(rb_entry_create_vld || rb_entry_ld_merge_vld){
    rb_entry_biu_pe_req_success := false.B
  }.elsewhen(rb_entry_biu_pe_req_grnt){
    rb_entry_biu_pe_req_success := true.B
  }

  //==========================================================
  //                 Generate create/cmit signal
  //==========================================================
  //------------------create read buffer signal---------------


}
