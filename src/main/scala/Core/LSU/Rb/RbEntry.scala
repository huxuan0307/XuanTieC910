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

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //-----------entry gateclk--------------
  //normal gateclk ,open when create || entry_vld




}
