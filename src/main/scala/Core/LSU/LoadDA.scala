package Core.LSU
import chisel3._
import chisel3.util._

class LoadDABorrowData extends Bundle{
  val db      = UInt(LSUConfig.VB_DATA_ENTRY.W)
  val vb      = Bool()
  val sndb    = Bool()
  val mmu     = Bool()
  val icc     = Bool()
  val icc_tag = Bool()
  val settle_way     = Bool()
}

class LoadDAInstData extends Bundle{
  val mmu_req                    = Bool()
  val expt_vld_except_access_err = Bool()
  val expt_access_fault_mask     = Bool()
  val expt_access_fault_extra    = Bool()
  val expt_access_fault_mmu      = Bool()
  val pfu_va                     = UInt(LSUConfig.PA_WIDTH.W)
  val split                      = Bool()
  val inst_type                  = UInt(2.W)
  val inst_size                  = UInt(3.W)
  val secd                       = Bool()
  val sign_extend                = Bool()
  val atomic                     = Bool()
  val iid                        = UInt(7.W)
  val lsid                       = Vec(LSUConfig.LSIQ_ENTRY, Bool())
  val boundary                   = Bool()
  val preg                       = UInt(7.W)
  val already_da                 = Bool()
  val ldfifo_pc                  = UInt(LSUConfig.PC_LEN.W)
  val ahead_predict              = Bool()
  val wait_fence                 = Bool()
  val other_discard_sq           = Bool()
  val data_discard_sq            = Bool()
  val fwd_sq_bypass              = Bool()
  val fwd_sq_vld                 = Bool()
  val data_vld_newest_fwd_sq_dup = Vec(4, Bool())
  val fwd_sq_multi               = Bool()
  val fwd_sq_multi_mask          = Bool()
  val fwd_bypass_sq_multi        = Bool()
  val sq_fwd_id                  = Vec(LSUConfig.SQ_ENTRY, Bool())
  val discard_wmb                = Bool()
  val fwd_wmb_vld                = Bool()
  val spec_fail                  = Bool()
  val bkpta_data                 = Bool()
  val bkptb_data                 = Bool()
  val vreg                       = UInt(6.W)
  val inst_vfls                  = Bool()
  val bytes_vld1                 = UInt(16.W)
  val fwd_bytes_vld              = UInt(16.W)
  val fwd_bytes_vld_dup          = UInt(16.W)
  val preg_sign_sel              = UInt(4.W)
  val vreg_sign_sel              = Bool()
  val cb_addr_create             = Bool()
  val cb_merge_en                = Bool()
  val pf_inst                    = Bool()
  val no_spec                    = Bool()
  val no_spec_exist              = Bool()
  val vector_nop                 = Bool()
}

class LoadDAShareData extends Bundle{
  val addr0        = UInt(LSUConfig.PA_WIDTH.W)
  val addr0_idx    = UInt(9.W)
  val old          = Bool()
  val page_so      = Bool()
  val page_ca      = Bool()
  val page_buf     = Bool()
  val page_sec     = Bool()
  val page_share   = Bool()
  val data_rot_sel = UInt(8.W)
  val bytes_vld    = UInt(16.W)
}

class LoadDA_DCHit extends Bundle{
  val dcache_hit          =  Bool()
  val hit_low_region      =  Bool()
  val hit_low_region_dup  =  Vec(4, Bool())
  val hit_high_region     =  Bool()
  val hit_high_region_dup =  Vec(4, Bool())

}


class LoadDA2SQ extends Bundle {
  val data_discard_vld = Bool()
  val fwd_id = UInt(12.W)
  val fwd_multi_vld = Bool()
  val global_discard_vld = Bool()
}

class LoadDA2CB extends Bundle{
  val data = UInt(128.W)
  val data_vld = Bool()
  val ecc_cancel = Bool()
  val ld_inst_vld = Bool()
}

class LoadDA2Mcic extends Bundle{
  val borrow_mmu_req = Bool()
  val bypass_data = UInt(64.W)
  val data_err = Bool()
  val rb_full = Bool()
  val wakeup = Bool()
}

class LoadDA2Ctrl extends Bundle{
  val borrow_vld              = Bool()
  val ecc_wakeup              = UInt(12.W)
  val idu_already_da          = UInt(12.W)
  val idu_bkpta_data          = UInt(12.W)
  val idu_bkptb_data          = UInt(12.W)
  val idu_boundary_gateclk_en = UInt(12.W)
  val idu_pop_entry           = UInt(12.W)
  val idu_pop_vld             = Bool()
  val idu_rb_full             = UInt(12.W)
  val idu_secd                = UInt(12.W)
  val idu_spec_fail           = UInt(12.W)
  val idu_wait_fence          = UInt(12.W)
  val rb_full_gateclk_en      = Bool()
  val special_gateclk_en      = Bool()
  val wait_fence_gateclk_en   = Bool()
}

class LoadDA2PFU extends Bundle{
  val ldfifo_pc = UInt(15.W)
  val page_sec_ff = Bool()
  val page_share_ff = Bool()

  val act_dp_vld = Bool()
  val act_vld = Bool()
  val biu_req_hit_idx = Bool()
  val evict_cnt_vld = Bool()
  val pf_inst_vld = Bool()
  val pfu_va  = UInt(40.W)
  val ppfu_va = UInt(40.W)

  val ppn_ff = UInt(28.W)
}
//
class LoadDA2RB extends {
  val boundary_after_mask = Bool()
  val bytes_vld = UInt(16.W)
  val data_ori = UInt(64.W)
  val data_rot_sel = UInt(8.W)
  val inst_size = UInt(3.W)
  val mcic_borrow_mmu = Bool()

  val old = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()

  val atomic = Bool()
  val cmit = Bool()
  val cmplt_success = Bool()
  val create_dp_vld = Bool()
  val create_gateclk_en = Bool()
  val create_judge_vld = Bool()
  val create_lfb = Bool()
  val create_vld = Bool()
  val data_vld = Bool()
  val dest_vld = Bool()
  val discard_grnt = Bool()
  val ldamo = Bool()
  val merge_dp_vld = Bool()
  val merge_expt_vld = Bool()
  val merge_gateclk_en = Bool()
  val merge_vld = Bool()

  val sign_extend = Bool()
  val vreg_sign_sel = Bool()
}

class LoadDA2WB extends Bundle{
  val preg_sign_sel = UInt(4.W)

  val cmplt_req = Bool()
  val data = UInt(64.W)
  val data_req = Bool()
  val data_req_gateclk_en = Bool()
  val expt_vec = UInt(5.W)
  val expt_vld = Bool()
  val mt_value = UInt(40.W)
  val no_spec_hit = Bool()
  val no_spec_mispred = Bool()
  val no_spec_miss = Bool()
  val spec_fail = Bool()
  val vreg_sign_sel = UInt(2.W)
}




class LoadDAInput extends Bundle{
  val cb_ld_da_data = Valid(UInt(128.W))
  val fromCp0 = new Bundle{
    val lsu_dcache_en = Bool()
    val lsu_icg_en = Bool()
    val lsu_l2_pref_en = Bool()
    val lsu_nsfe = Bool()
    val yy_clk_en = Bool()
    val yy_dcache_pref_en = Bool()
  }
  val ctrl_ld_clk = Bool()
  val dcache_lsu_ld_data_bank_dout = Vec(8, UInt(32.W))
  val fromDC = new LoadDC2DA
  val ld_hit_prefetch = Bool()
  val lfb_ld_da_hit_idx = Bool()
  val lm_ld_da_hit_idx = Bool()
  val lsu_special_clk = Bool()
  val mmu_lsu_access_fault0 = Bool()
  val fromPad = new Bundle{
    val yy_icg_scan_en = Bool()
  }
  val pfu_biu_req_addr = UInt(40.W)
  val fromRB = new Bundle{
    val full = Bool()
    val hit_idx = Bool()
    val merge_fail = Bool()
  }
  val fromRTU = new Bundle{
    val yy_xx_commit = Vec(3, Bool())
    val yy_xx_commit_iid = Vec(3, UInt(7.W))
    val yy_xx_flush = Bool()
  }
  val fromSdEx1 = new Bundle{
    val data_bypass = UInt(128.W)
    val inst_vld = Bool()
  }
  val fromSF = new Bundle{
    val spec_hit = Bool()
    val spec_mark = Bool()
  }
  val fromSQ = new Bundle{
    val fwd_data = UInt(64.W)
    val fwd_data_pe = UInt(64.W)
    val data_discard_req = Bool()
    val fwd_bypass_multi = Bool()
    val fwd_bypass_req = Bool()
    val fwd_id = UInt(12.W)
    val fwd_multi = Bool()
    val fwd_multi_mask = Bool()
    val newest_fwd_data_vld_req = Bool()
    val other_discard_req = Bool()
  }
  val st_da_addr = UInt(40.W)
  val fromWmb = new Bundle{
    val ld_da_fwd_data = UInt(128.W)
    val ld_dc_discard_req = Bool()
  }
}

class LoadDAOutput extends Bundle{
  val toCtrl = new LoadDA2Ctrl
  val toPFU  = new LoadDA2PFU
  val toRB   = new LoadDA2RB
  val toWB   = new LoadDA2WB
  val toSQ   = new LoadDA2SQ
  val toCB   = new LoadDA2CB
  val toMcic = new LoadDA2Mcic
  val toLfb = new Bundle {
    val discard_grnt = Bool()
    val set_wakeup_queue = Bool()
    val wakeup_queue_next = UInt(13.W)
  }
  val toLm = new Bundle{
    val discard_grnt = Bool()
    val ecc_err = Bool()
    val no_req = Bool()
    val vector_nop = Bool()
  }
  val toSF = new Bundle{
    val addr_tto4 = UInt(36.W)
    val bytes_vld = UInt(16.W)
    val spec_chk_req = Bool()
  }
  val ld_da_addr = UInt(40.W)
  val ld_da_bkpta_data = Bool()
  val ld_da_bkptb_data = Bool()
  val ld_da_data256 = UInt(256.W)
  val ld_da_dcache_hit = Bool()
  val ld_da_fwd_ecc_stall = Bool()
  val ld_da_icc_read_data = UInt(128.W)
  val ld_da_idx = UInt(8.W)
  val ld_da_iid = UInt(7.W)
  val ld_da_inst_vfls = Bool()
  val ld_da_inst_vld = Bool()
  val ld_da_lsid = UInt(12.W)
  val ld_da_preg = UInt(7.W)
  val ld_da_snq_borrow_icc = Bool()
  val ld_da_snq_borrow_sndb = Bool()
  val ld_da_st_da_hit_idx = Bool()
  val ld_da_vb_borrow_vb = UInt(3.W)
  val ld_da_vb_snq_data_reissue = Bool()
  val ld_da_vreg = UInt(6.W)
  val ld_da_wmb_discard_vld = Bool()
  val toHpcp = new Bundle{
    val ld_cache_access = Bool()
    val ld_cache_miss = Bool()
    val ld_data_discard = Bool()
    val ld_discard_sq = Bool()
    val ld_unalign_inst = Bool()
  }
  val toIDU = new Bundle{
    val da_pipe3_fwd_pre = UInt(7.W)
    val da_pipe3_fwd_preg_dat = UInt(64.W)
    val da_pipe3_fwd_preg_vld = Bool()
    val da_pipe3_fwd_vreg = UInt(7.W)
    val da_pipe3_fwd_vreg_fr_data = UInt(64.W)
    val da_pipe3_fwd_vreg_vld = Bool()
    val da_pipe3_fwd_vreg_vr0_data = UInt(64.W)
    val da_pipe3_fwd_vreg_vr1_data = UInt(64.W)
    val ld_da_wait_old = UInt(12.W)
    val ld_da_wait_old_gateclk_en = Bool()
  }
  val toRTU = new Bundle{
    val da_pipe3_split_spec_fail_iid = UInt(7.W)
    val da_pipe3_split_spec_fail_vld = Bool()
  }
}

class LoadDAIO extends Bundle{
  val in  = Input(new LoadDAInput)
  val out = Output(new LoadDAOutput)
}

class LoadDA extends Module {
  val io = IO(new LoadDAIO)

  //Reg
  val ld_da_inst_vld          = RegInit(false.B)
  val ld_da_ahead_preg_wb_vld = RegInit(false.B)
  val ld_da_ahead_vreg_wb_vld = RegInit(false.B)
  val ld_da_borrow_vld        = RegInit(false.B)

  val ld_da_dcache_data_bank = Reg(Vec(8, UInt(32.W)))
  val ld_da_tag_read = Reg(UInt(27.W))

  val ld_da_expt_vec = RegInit(0.U(5.W))
  val ld_da_mt_value = RegInit(0.U(LSUConfig.PA_WIDTH.W))

  val borrow_data = RegInit(0.U.asTypeOf(new LoadDABorrowData))
  val inst_data = RegInit(0.U.asTypeOf(new LoadDAInstData))
  val share_data = RegInit(0.U.asTypeOf(new LoadDAShareData))
  val dcache_hit_info = RegInit(0.U.asTypeOf(new LoadDA_DCHit))
  val ld_da_ppfu_va = RegInit(0.U(LSUConfig.PA_WIDTH.W))
  //Wire
  val ld_da_ecc_stall_gate = Wire(Bool())
  val ld_da_ecc_stall = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //------------------normal reg------------------------------
  val ld_da_clk_en = io.in.fromDC.inst_vld || ld_da_ecc_stall_gate || io.in.fromDC.borrow_vld && io.in.fromDC.borrow_mmu

  val ld_da_inst_clk_en = io.in.fromDC.inst_vld

  val ld_da_borrow_clk_en = io.in.fromDC.borrow_vld

  val ld_da_expt_clk_en = io.in.fromDC.inst_vld && io.in.fromDC.da_expt_vld_gate_en || ld_da_ecc_stall_gate

  //------------------pfu_addr reg----------------------------
  val ld_da_pfu_info_clk_en = io.in.fromDC.pfu_info_set_vld

  //------------------dcache reg------------------------------
  val ld_da_data_clk_en = io.in.fromDC.get_dcache_data.asBools.foreach(dc_vld => dc_vld || ld_da_ecc_stall_gate)

  val ld_da_tag_clk_en = io.in.fromDC.da_icc_tag_vld

  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------control part----------------------------
  //+----------+------------+
  //| inst_vld | borrow_vld |
  //+----------+------------+
  when(io.in.fromDC.da_inst_vld){
    ld_da_inst_vld := true.B
    ld_da_ahead_preg_wb_vld := io.in.fromDC.ahead_preg_wb_vld
    ld_da_ahead_vreg_wb_vld := io.in.fromDC.ahead_vreg_wb_vld
  }.otherwise{
    ld_da_inst_vld := false.B
    ld_da_ahead_preg_wb_vld := false.B
    ld_da_ahead_vreg_wb_vld := false.B
  }

  when(io.in.fromDC.borrow_vld){
    ld_da_borrow_vld := true.B
  }.otherwise{
    ld_da_borrow_vld := false.B
  }

  val ld_da_ecc_stall_already = false.B
  val ld_da_ecc_stall_fatal   = false.B

  //+----------+----------+----------+----------+
  //| data 0/4 | data 1/5 | data 2/6 | data 3/7 |
  //+----------+----------+----------+----------+
  //data bank0 and bank4 use a common gateclk because they are read
  //simultaneously in all case
  ld_da_dcache_data_bank := io.in.dcache_lsu_ld_data_bank_dout

  //------------------tag read for debug-------------------------------
  when(io.in.fromDC.da_icc_tag_vld){
    ld_da_tag_read := io.in.fromDC.da_tag_read
  }

  //------------------expt part-------------------------------
  //+----------+
  //| expt_vec |
  //+----------+
  when(io.in.fromDC.inst_vld && io.in.fromDC.da_expt_vld_gate_en && !ld_da_ecc_stall){
    ld_da_expt_vec := io.in.fromDC.expt_vec
    ld_da_mt_value := io.in.fromDC.mt_value
  }

  //------------------borrow part-----------------------------
  //+-----+------+-----+------------+
  //| vb | sndb | mmu | settle way |
  //+-----+------+-----+------------+
  when(io.in.fromDC.borrow_vld && !ld_da_ecc_stall){
    borrow_data.db         :=  io.in.fromDC.borrow_db
    borrow_data.vb         :=  io.in.fromDC.borrow_vb
    borrow_data.sndb       :=  io.in.fromDC.borrow_sndb
    borrow_data.mmu        :=  io.in.fromDC.borrow_mmu
    borrow_data.icc        :=  io.in.fromDC.borrow_icc
    borrow_data.icc_tag    :=  io.in.fromDC.borrow_icc_tag
    borrow_data.settle_way :=  io.in.fromDC.settle_way
  }

  //------------------inst part----------------------------
  //+----------+
  //| expt_vld |
  //+----------+
  //+-----------+-----------+------+-----------+
  //| inst_type | inst_size | secd | inst_zone |
  //+-----------+-----------+------+-----------+
  //+-------------+----+-----+------+-----+------------+------------+
  //| sign_extend | ex | iid | lsid | old | bytes_vld0 | bytes_vld1 |
  //+-------------+----+-----+------+-----+------------+------------+
  //+----------+------+---------------+-------+
  //| boundary | preg | rar_spec_fail | split |
  //+----------+------+---------------+-------+
  //+------------+-----------+-------+-------+
  //| already_da | ldfifo_pc | bkpta | bkptb |
  //+------------+-----------+-------+-------+
  //+----+----+-----+-----+-------+
  //| so | ca | buf | sec | share |
  //+----+----+-----+-----+-------+
  //+------------+-------------+-----------+------------+
  //| fwd_sq_vld | fwd_wmb_vld | sq_fwd_id | wmb_fwd_id |
  //+------------+-------------+-----------+------------+
  //+--------------+----------------+-----------------+
  //| sq_fwd_multi | sq_discard_req | wmb_discard_req |
  //+--------------+----------------+-----------------+
  when(io.in.fromDC.inst_vld && !ld_da_ecc_stall){
    inst_data.mmu_req                    :=  io.in.fromDC.mmu_req
    inst_data.expt_vld_except_access_err :=  io.in.fromDC.expt_vld_except_access_err
    inst_data.expt_access_fault_mask     :=  io.in.fromDC.expt_access_fault_mask
    inst_data.expt_access_fault_extra    :=  io.in.fromDC.expt_access_fault_extra
    inst_data.expt_access_fault_mmu      :=  io.in.mmu_lsu_access_fault0
    inst_data.pfu_va                     :=  io.in.fromDC.pfu_va
    inst_data.split                      :=  io.in.fromDC.split
    inst_data.inst_type                  :=  io.in.fromDC.inst_type
    inst_data.inst_size                  :=  io.in.fromDC.inst_size
    inst_data.secd                       :=  io.in.fromDC.secd
    inst_data.sign_extend                :=  io.in.fromDC.sign_extend
    inst_data.atomic                     :=  io.in.fromDC.atomic
    inst_data.iid                        :=  io.in.fromDC.iid
    inst_data.lsid                       :=  io.in.fromDC.lsid
    inst_data.boundary                   :=  io.in.fromDC.boundary
    inst_data.preg                       :=  io.in.fromDC.preg
    inst_data.already_da                 :=  io.in.fromDC.already_da
    inst_data.ldfifo_pc                  :=  io.in.fromDC.ldfifo_pc
    inst_data.ahead_predict              :=  io.in.fromDC.ahead_predict
    inst_data.wait_fence                 :=  io.in.fromDC.wait_fence
    inst_data.other_discard_sq           :=  io.in.fromSQ.other_discard_req
    inst_data.data_discard_sq            :=  io.in.fromSQ.data_discard_req
    inst_data.fwd_sq_bypass              :=  io.in.fromSQ.fwd_bypass_req
    inst_data.fwd_sq_vld                 :=  io.in.fromDC.fwd_sq_vld
    inst_data.data_vld_newest_fwd_sq_dup :=  VecInit(Seq.fill(4)(io.in.fromSQ.newest_fwd_data_vld_req))
    inst_data.fwd_sq_multi              :=  io.in.fromSQ.fwd_multi
    inst_data.fwd_sq_multi_mask         :=  io.in.fromSQ.fwd_multi_mask
    inst_data.fwd_bypass_sq_multi       :=  io.in.fromSQ.fwd_bypass_multi
    inst_data.sq_fwd_id                 :=  io.in.fromSQ.fwd_id
    inst_data.discard_wmb               :=  io.in.fromWmb.ld_dc_discard_req
    inst_data.fwd_wmb_vld               :=  io.in.fromDC.fwd_wmb_vld
    inst_data.spec_fail                 :=  io.in.fromDC.spec_fail
    inst_data.bkpta_data                :=  io.in.fromDC.bkpta_data
    inst_data.bkptb_data                :=  io.in.fromDC.bkptb_data
    inst_data.vreg                      :=  io.in.fromDC.vreg
    inst_data.inst_vfls                 :=  io.in.fromDC.inst_vfls
    inst_data.bytes_vld1                :=  io.in.fromDC.da_bytes_vld1
    inst_data.fwd_bytes_vld             :=  io.in.fromDC.fwd_bytes_vld
    inst_data.fwd_bytes_vld_dup         :=  io.in.fromDC.fwd_bytes_vld
    inst_data.preg_sign_sel             :=  io.in.fromDC.preg_sign_sel
    inst_data.vreg_sign_sel             :=  io.in.fromDC.vreg_sign_sel
    inst_data.cb_addr_create            :=  io.in.fromDC.da_cb_addr_create
    inst_data.cb_merge_en               :=  io.in.fromDC.da_cb_merge_en
    inst_data.pf_inst                   :=  io.in.fromDC.da_pf_inst
    inst_data.no_spec                   :=  io.in.fromDC.no_spec
    inst_data.no_spec_exist             :=  io.in.fromDC.no_spec_exist
    inst_data.vector_nop                :=  io.in.fromDC.vector_nop
  }

  //------------------inst/borrow share part------------------
  //+------+-----------------+------------------+
  //| addr | dcache_data_sel | page_information |
  //+------+-----------------+------------------+
  when((io.in.fromDC.inst_vld || io.in.fromDC.borrow_vld && !io.in.fromDC.borrow_vb && !io.in.fromDC.borrow_sndb) && !ld_da_ecc_stall){
    share_data.addr0        :=  io.in.fromDC.addr0
    share_data.addr0_idx    :=  io.in.fromDC.addr0
    share_data.old          :=  io.in.fromDC.da_old
    share_data.page_so      :=  io.in.fromDC.da_page_so
    share_data.page_ca      :=  io.in.fromDC.da_page_ca
    share_data.page_buf     :=  io.in.fromDC.da_page_buf
    share_data.page_sec     :=  io.in.fromDC.da_page_sec
    share_data.page_share   :=  io.in.fromDC.da_page_share
    share_data.data_rot_sel :=  io.in.fromDC.da_data_rot_sel
    share_data.bytes_vld    :=  io.in.fromDC.da_bytes_vld
  }

  when((io.in.fromDC.inst_vld || io.in.fromDC.borrow_vld && io.in.fromDC.borrow_mmu) && !ld_da_ecc_stall){
    dcache_hit_info.dcache_hit           :=  io.in.fromDC.dcache_hit
    dcache_hit_info.hit_low_region       :=  io.in.fromDC.hit_low_region
    dcache_hit_info.hit_low_region_dup   :=  VecInit(Seq.fill(4)(io.in.fromDC.hit_low_region))
    dcache_hit_info.hit_high_region      :=  io.in.fromDC.hit_high_region
    dcache_hit_info.hit_high_region_dup  :=  VecInit(Seq.fill(4)(io.in.fromDC.hit_high_region))
  }

  //+----------+
  //| pfu_addr |
  //+----------+
  //low power pfu_addr, reverse only when pfb/sdb not empty
  when(io.in.fromDC.pfu_info_set_vld && !ld_da_ecc_stall){
    ld_da_ppfu_va := io.in.fromDC.pfu_va
  }

  //==========================================================
  //        Generate expt info
  //==========================================================


}
