package Core.LSU.Rb
import Core.LsuConfig
import chisel3._
import chisel3.util._

class RbFromCp0 extends Bundle{
  val lsu_dcache_en = Bool()
  val lsu_icg_en = Bool()
  val yy_clk_en = Bool()
  val yy_priv_mode = UInt(2.W)
}

class RbFromLoadDA extends Bundle with LsuConfig{
  val addr = UInt(PA_WIDTH.W)
  val bkpta_data = Bool()
  val bkptb_data = Bool()
  val boundary_after_mask = Bool()
  val bytes_vld = UInt(16.W)
  val data_ori = UInt(64.W)
  val data_rot_sel = UInt(8.W)
  val dcache_hit = Bool()
  val idx = UInt(8.W)
  val iid = UInt(7.W)
  val inst_size = UInt(3.W)
  val inst_vfls = Bool()
  val mcic_borrow_mmu = Bool()
  val old = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val preg = UInt(7.W)
  val rb_atomic = Bool()
  val rb_cmit = Bool()
  val rb_cmplt_success = Bool()
  val rb_create_dp_vld = Bool()
  val rb_create_gateclk_en = Bool()
  val rb_create_judge_vld = Bool()
  val rb_create_lfb = Bool()
  val rb_create_vld = Bool()
  val rb_data_vld = Bool()
  val rb_dest_vld = Bool()
  val rb_discard_grnt = Bool()
  val rb_ldamo = Bool()
  val rb_merge_dp_vld = Bool()
  val rb_merge_expt_vld = Bool()
  val rb_merge_gateclk_en = Bool()
  val rb_merge_vld = Bool()
  val sign_extend = Bool()
  val vreg = UInt(6.W)
  val vreg_sign_sel = Bool()
}

class RbFromStoreDA extends Bundle with LsuConfig {
  val addr = UInt(PA_WIDTH.W)
  val dcache_hit = Bool()
  val fence_inst = Bool()
  val fence_mode = UInt(4.W)
  val iid = UInt(7.W)
  val inst_size = UInt(3.W)
  val old = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val rb_cmit = Bool()
  val rb_create_dp_vld = Bool()
  val rb_create_gateclk_en = Bool()
  val rb_create_lfb = Bool()
  val rb_create_vld = Bool()
  val sync_fence = Bool()
  val sync_inst = Bool()
}

class RbFromWmb extends Bundle with LsuConfig {
  val ce_addr = UInt(PA_WIDTH.W)
  val ce_page_ca = Bool()
  val ce_page_so = Bool()
  val has_sync_fence = Bool()
  val rb_biu_req_hit_idx = Bool()
  val rb_so_pending = Bool()
  val sync_fence_biu_req_success = Bool()
}

class RbFromRTU extends Bundle {
  val lsu_async_flush = Bool()
  val yy_xx_commit = Vec(3, Bool())
  val yy_xx_commit_iid = Vec(3, UInt(7.W))
  val yy_xx_flush = Bool()
}







class Rb {

}
