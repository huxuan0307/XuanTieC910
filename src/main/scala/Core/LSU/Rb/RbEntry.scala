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
    val b_id        = UInt(5.W)
    val b_vld       = Bool()
    val r_data_mask = UInt(128.W)
    val r_id        = UInt(5.W)
    val r_vld       = Bool()
  }
  val fromCp0 = new RbFromCp0
  val fromLoadDA = new RbFromLoadDA

  val lm_already_snoop   = Bool()
  val lsu_has_fence      = Bool()
  val lsu_special_clk    = Bool()
  val pad_yy_icg_scan_en = Bool()
  val pfu_biu_req_addr   = UInt(PA_WIDTH.W)
  val rb_biu_ar_id       = UInt(5.W)

  val RbEntry = new RbEntryIn

  val rb_fence_ld                  = Bool()
  val rb_ld_biu_pe_req_grnt        = Bool()
  val rb_r_resp_err                = Bool()
  val rb_r_resp_okay               = Bool()
  val rb_ready_all_req_biu_success = Bool()
  val rb_ready_ld_req_biu_success  = Bool()

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
  def IDLE        = "b0000".U(4.W)
  def WAIT_RDY    = "b1000".U(4.W)
  def REQ_BIU     = "b1001".U(4.W)
  def WAIT_RESP   = "b1100".U(4.W)
  def REQ_WB      = "b1101".U(4.W)
  def WAIT_MERGE  = "b1110".U(4.W)

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

  val rb_entry_next_state = WireInit(IDLE)

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

  val rb_entry_ld_create_vld = Wire(Bool())
  val rb_entry_st_create_vld = Wire(Bool())

  val rb_entry_create_wait_rdy = Wire(Bool())
  val rb_entry_ready_to_biu_req = Wire(Bool())
  val rb_entry_wait_resp_imme_idle = Wire(Bool())
  val rb_entry_sync_fence_resp_success = Wire(Bool())
  val rb_entry_wait_resp_to_req_merge = Wire(Bool())
  val rb_entry_req_wb_success = Wire(Bool())

  val rb_entry_biu_req = Wire(Bool())
  val rb_entry_next_nc_bypass = Wire(Bool())
  val rb_entry_next_so_bypass = Wire(Bool())
  val rb_entry_ld_da_hit_idx = Wire(Bool())
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
  rb_entry_create_dp_vld := rb_entry_ld_create_dp_vld || rb_entry_st_create_dp_vld

  rb_entry_create_vld    := rb_entry_ld_create_vld || rb_entry_st_create_vld
  //------------------commit set signal-----------------------
  val rb_entry_cmit_hit = Wire(Vec(3, Bool()))
  for(i <- 0 until 3){
    rb_entry_cmit_hit(i) := io.in.fromRTU.yy_xx_commit(i) && io.in.fromRTU.yy_xx_commit_iid(i) === inst_info.iid
  }

  rb_entry_cmit_set := rb_entry_cmit_hit.asUInt.orR && rb_entry_vld

  //==========================================================
  //                 Generate inst type
  //==========================================================
  val rb_entry_sync_fence = inst_info.sync ||  inst_info.fence
  val rb_entry_not_sync_fence = !rb_entry_sync_fence

  //==========================================================
  //                 Generate next state
  //==========================================================
  switch(rb_entry_state){
    is(IDLE){
      when(rb_entry_ld_create_vld && io.in.fromLoadDA.boundary_after_mask && io.in.fromLoadDA.rb_data_vld){
        rb_entry_next_state := WAIT_MERGE
      }.elsewhen(rb_entry_create_wait_rdy){
        rb_entry_next_state := WAIT_RDY
      }.elsewhen(rb_entry_create_vld){
        rb_entry_next_state := REQ_BIU
      }.otherwise{
        rb_entry_next_state := IDLE
      }
    }

    is(WAIT_RDY){
      when(rb_entry_flush_clear){
        rb_entry_next_state := IDLE
      }.elsewhen(rb_entry_ready_to_biu_req){
        rb_entry_next_state := REQ_BIU
      }.otherwise{
        rb_entry_next_state := WAIT_RDY
      }
    }

    is(REQ_BIU){
      when(rb_entry_flush_clear){
        rb_entry_next_state := IDLE
      }.elsewhen(rb_entry_read_req_grnt){
        rb_entry_next_state := WAIT_RESP
      }.otherwise{
        rb_entry_next_state := REQ_BIU
      }
    }

    is(WAIT_RESP){
      when(rb_entry_wait_resp_imme_idle || rb_entry_sync_fence_resp_success || io.in.fromRTU.lsu_async_flush){
        rb_entry_next_state := IDLE
      }.elsewhen(rb_entry_wait_resp_to_req_merge){
        rb_entry_next_state := WAIT_MERGE
      }.elsewhen(rb_entry_biu_r_resp_set && rb_entry_not_sync_fence){
        rb_entry_next_state := REQ_WB
      }.otherwise{
        rb_entry_next_state := WAIT_RESP
      }
    }

    is(REQ_WB){
      when(!rb_entry_dest_vld || rb_entry_req_wb_success || io.in.fromRTU.lsu_async_flush){
        rb_entry_next_state := IDLE
      }.otherwise{
        rb_entry_next_state := REQ_WB
      }
    }

    is(WAIT_MERGE){
      when(!rb_entry_dest_vld ||  rb_entry_flush_clear){
        rb_entry_next_state := IDLE
      }.elsewhen(rb_entry_ld_merge_vld && io.in.fromLoadDA.rb_data_vld){
        rb_entry_next_state := REQ_WB
      }.elsewhen(rb_entry_ld_merge_vld){
        rb_entry_next_state := REQ_BIU
      }.otherwise{
        rb_entry_next_state := WAIT_MERGE
      }
    }
  }

  //==========================================================
  //                 State 0 : idle
  //==========================================================
  //create ptr0 is used for ld pipe
  //create ptr1 is used for st pipe
  val rb_entry_hit_fence_ld = io.in.lsu_has_fence && io.in.rb_fence_ld && io.in.RbEntry.create_ptr0

  rb_entry_create_wait_rdy := rb_entry_ld_create_vld && !io.in.fromLoadDA.mcic_borrow_mmu && (io.in.fromLoadDA.page_so || rb_entry_hit_fence_ld) ||
    rb_entry_st_create_vld && (io.in.fromStoreDA.fence_inst || io.in.fromStoreDA.sync_inst)

  //==========================================================
  //                 State 1 : wait for ready
  //==========================================================
  //to make it easy, rb send bar/sync ar request must wait wmb has sent aw request
  val rb_entry_fence_ready = inst_info.fence && io.in.fromWmb.sync_fence_biu_req_success && (!inst_info.fence_mode(0) || io.in.rb_ready_ld_req_biu_success)

  val rb_entry_sync_ready  = inst_info.sync && io.in.fromWmb.sync_fence_biu_req_success && io.in.rb_ready_all_req_biu_success

  val rb_entry_not_sync_fence_ready = rb_entry_not_sync_fence &&
    (inst_info.page_so && rb_entry_cmit && !io.in.fromWmb.rb_so_pending || !inst_info.page_so && !io.in.lsu_has_fence)

  rb_entry_ready_to_biu_req := rb_entry_vld &&
    (rb_entry_fence_ready || rb_entry_sync_ready || rb_entry_not_sync_fence_ready)

  val rb_entry_biu_pe_req_gateclk_en =
    (rb_entry_state === WAIT_RDY || rb_entry_biu_req && !rb_entry_biu_pe_req_success) && !rb_entry_flush_clear

  val rb_entry_biu_pe_req = (rb_entry_ready_to_biu_req && (rb_entry_state ===  WAIT_RDY) ||
    rb_entry_biu_req && !rb_entry_biu_pe_req_success) && !rb_entry_flush_clear

  //==========================================================
  //                 State 2 : request biu/lfb
  //==========================================================
  //------------------biu/lfb req-----------------------------
  rb_entry_biu_req := rb_entry_state === REQ_BIU
  io.out.biu_req := rb_entry_biu_req

  //if request both biu and lfb, then the rb_entry must get two grnt signal, else
  //it will not request biu or lfb. it is realized in rb top module.

  //rb_entry_read_req_grnt is an input signal and will not generate in the entry.
  //==========================================================
  //                 State 3 : wait data/resp
  //==========================================================
  //------------------biu response signal---------------------
  val rb_entry_r_id_hit = io.in.fromBiu.r_vld && rb_entry_biu_id === io.in.fromBiu.r_id

  val rb_entry_b_id_hit = io.in.fromBiu.b_vld && rb_entry_biu_id === io.in.fromBiu.b_id

  //-----------biu response signal--------
  rb_entry_biu_r_resp_set := rb_entry_r_id_hit && (rb_entry_state ===  WAIT_RESP) &&
    (inst_info.page_ca || inst_info.atomic && !inst_info.page_so || rb_entry_next_nc_bypass || rb_entry_next_so_bypass || rb_entry_sync_fence)

  rb_entry_biu_b_resp_set := rb_entry_b_id_hit

  val rb_entry_atomic_next_resp  = rb_entry_vld && inst_info.atomic && (rb_entry_next_so_bypass || !inst_info.page_so)

  //if non-cacheable ldex, response okay is regarded as bus error
  rb_entry_bus_err_set := rb_entry_biu_r_resp_set && (io.in.rb_r_resp_err || inst_info.atomic && !inst_info.page_ca && io.in.rb_r_resp_okay)

  rb_entry_fof_bus_err_expt := false.B
  rb_entry_fof_not_first    := false.B
  //------------------data bypass signal----------------------
  rb_entry_data_bypass_vld := rb_entry_biu_r_resp_set && rb_entry_dest_vld &&
    (!inst_info.atomic || io.in.lm_already_snoop || !inst_info.dcache_hit)

  //------------------settle data from biu--------------------
  val rb_entry_biu_data_ori = Wire(Vec(16, UInt(8.W)))
  for(i <- 0 until 16){
    rb_entry_biu_data_ori(i) := Mux(inst_info.bytes_vld(i), io.in.fromBiu.r_data_mask(8*i+7,8*i), 0.U(8.W))
  }

  rb_entry_biu_data_update := rb_entry_biu_data_ori.asUInt(127,64) | rb_entry_biu_data_ori.asUInt(63,0)
  //---------------------merge data---------------------------
  val rb_entry_merge_sel = rb_entry_state === WAIT_MERGE
  val rb_entry_merge_data_ori = Mux(rb_entry_merge_sel, io.in.fromLoadDA.data_ori, rb_entry_biu_data_update)

  val rb_entry_merge_bytes_vld = Mux(rb_entry_merge_sel,
    io.in.fromLoadDA.bytes_vld(15,8) | io.in.fromLoadDA.bytes_vld(7,0),
    inst_info.bytes_vld(15,8) | inst_info.bytes_vld(7,0))

  val rb_entry_merge_data_sel = Wire(Vec(8, UInt(8.W)))
  for(i <- 0 until 8){
    rb_entry_merge_data_sel(i) := VecInit(Seq.fill(8)(rb_entry_merge_bytes_vld(i))).asUInt
  }

  rb_entry_merge_data := rb_entry_data & ~rb_entry_merge_data_sel.asUInt | rb_entry_merge_data_ori

  //------------------resp success signal---------------------
  // inst_type                      cmplt condition
  // sync                           r&b resp
  // fence                          r&b resp
  // ld/atomic !boundary_first      -> REQ_WB
  // ld boundary_first              -> WAIT_MERGE
  val rb_entry_sync_resp_success = inst_info.sync && rb_entry_biu_r_resp && rb_entry_biu_b_resp

  val rb_entry_fence_resp_success = inst_info.fence && rb_entry_biu_r_resp && rb_entry_biu_b_resp

  //------------------generate next state signal--------------
  rb_entry_wait_resp_imme_idle := !rb_entry_dest_vld && inst_info.page_ca && rb_entry_not_sync_fence

  rb_entry_sync_fence_resp_success := rb_entry_sync_resp_success || rb_entry_fence_resp_success

  rb_entry_wait_resp_to_req_merge  := rb_entry_biu_r_resp_set && inst_info.boundary && !rb_entry_secd && rb_entry_not_sync_fence

  //==========================================================
  //                 State 4 : req cmplt/data
  //==========================================================
  //------------------req cmplt signal------------------------
  //so need to request wb cmplt part when grnt
  //ldex need to request wb cmplt part when get data
  //and only one entry will request cmplt part
  val rb_entry_wb_cmplt_req = !rb_entry_wb_cmplt_success && rb_entry_dest_vld &&
    (inst_info.page_so && !inst_info.atomic && (rb_entry_state ===  WAIT_RESP) || (rb_entry_state ===  REQ_WB))

  //------------------req data signal-------------------------
  //if get bus error, then it must commit and send bus_err signal
  val rb_entry_wb_data_req = rb_entry_dest_vld && (rb_entry_state === REQ_WB) &&
    (!rb_entry_bus_err || rb_entry_cmit) && rb_entry_wb_cmplt_success && !rb_entry_wb_data_success

  //for timing, select data_ptr one cycle ahead
  val rb_entry_wb_data_req_pre = rb_entry_biu_r_resp_set && (!inst_info.boundary || rb_entry_secd) &&
    rb_entry_wb_cmplt_success && rb_entry_not_sync_fence

  val rb_entry_wb_data_pre_sel = rb_entry_wb_data_req || rb_entry_wb_data_req_pre

  //------------------generate next state signal--------------
  rb_entry_req_wb_success := rb_entry_vld && rb_entry_wb_cmplt_success && (rb_entry_wb_data_grnt || rb_entry_wb_data_success)

  //get data_vld signal, if data_vld, then it will not hit idx on sq_pop
  val rb_entry_data_vld = (rb_entry_state === REQ_WB) || (rb_entry_state === WAIT_MERGE)

  //==========================================================
  //                 State 5 : wait for merge
  //==========================================================
  //------------------generate merge signal------------------
  val rb_entry_iid_hit = inst_info.iid === io.in.fromLoadDA.iid
  val rb_entry_ld_merge_pre = (rb_entry_state === WAIT_MERGE) && rb_entry_iid_hit

  rb_entry_ld_merge_vld := io.in.fromLoadDA.rb_merge_vld && rb_entry_ld_merge_pre

  rb_entry_ld_merge_dp_vld := io.in.fromLoadDA.rb_merge_dp_vld && rb_entry_ld_merge_pre

  //this signal is for cancel dest_vld and pop entry
  rb_entry_ld_merge_expt_vld :=
    io.in.fromLoadDA.rb_merge_expt_vld && rb_entry_vld && inst_info.boundary && !rb_entry_secd && rb_entry_iid_hit

  rb_entry_ld_merge_gateclk_en := io.in.fromLoadDA.rb_merge_gateclk_en && rb_entry_ld_merge_pre

  //------------------boundary depd vld-----------------------
  val rb_entry_merge_fail = rb_entry_iid_hit && rb_entry_dest_vld && inst_info.boundary &&
    !rb_entry_secd && rb_entry_vld && (rb_entry_state =/=  WAIT_MERGE)

  rb_entry_boundary_depd_set := rb_entry_merge_fail && io.in.fromLoadDA.rb_discard_grnt && !rb_entry_ld_da_hit_idx

  //------------------boundary depd clr-----------------------
  rb_entry_boundary_wakeup := rb_entry_boundary_depd && (rb_entry_state === WAIT_MERGE)
  io.out.boundary_wakeup

  //==========================================================
  //                 Barrier inst
  //==========================================================
  val rb_entry_fence_ld_vld = inst_info.fence && rb_entry_vld && inst_info.fence_mode(0)

  //==========================================================
  //                 Compare index
  //==========================================================
  //------------------compare ld_da stage---------------------
  //if has requested biu, then it will not compare with ld_da/st_da
  rb_entry_ld_da_hit_idx := inst_info.page_ca && rb_entry_vld && !rb_entry_sync_fence &&
    !rb_entry_biu_req_success && (inst_info.addr(13,6) === io.in.fromLoadDA.idx(7,0))

  //------------------compare st_da stage---------------------
  val rb_entry_st_da_hit_idx = inst_info.page_ca && rb_entry_vld && !rb_entry_biu_req_success &&
    !rb_entry_sync_fence && (inst_info.addr(13,6) === io.in.fromStoreDA.addr(13,6))

  //------------------depd_vld--------------------------------
  rb_entry_discard_vld := io.in.fromLoadDA.rb_discard_grnt && rb_entry_ld_da_hit_idx

  //------------------compare sq pop entry--------------------
  //sq pop entry must compare index if:
  //ca && !req_biu
  //!ca && !so && !data_vld
  val rb_entry_cmp_sq_pop_addr = io.in.fromSQ.pop_addr
  val rb_entry_sq_pop_cmp_vld  = inst_info.page_ca && io.in.fromSQ.pop_page_ca && !rb_entry_biu_req_success ||
    !inst_info.page_ca && !io.in.fromSQ.pop_page_ca && !inst_info.page_so && !io.in.fromSQ.pop_page_so && !rb_entry_data_vld

  val rb_entry_sq_pop_hit_idx  = rb_entry_sq_pop_cmp_vld && rb_entry_vld && !rb_entry_sync_fence &&
    rb_entry_cmit && (inst_info.addr(13,6)  === rb_entry_cmp_sq_pop_addr(13,6))

  //------------------compare wmb ce entry---------------------
  val rb_entry_cmp_wmb_ce_addr = io.in.fromWmb.ce_addr
  val rb_entry_wmb_ce_cmp_vld  = inst_info.page_ca && io.in.fromWmb.ce_page_ca && !rb_entry_biu_req_success ||
    !inst_info.page_ca && !io.in.fromWmb.ce_page_ca && !inst_info.page_so && !io.in.fromWmb.ce_page_so && !rb_entry_data_vld

  val rb_entry_wmb_ce_hit_idx  = rb_entry_wmb_ce_cmp_vld && rb_entry_vld && !rb_entry_sync_fence &&
  rb_entry_cmit && (inst_info.addr(13,6) === rb_entry_cmp_wmb_ce_addr(13,6))

  //------------------compare pfu pop entry--------------------
  val rb_entry_cmp_pfu_biu_req_addr = io.in.pfu_biu_req_addr
  val rb_entry_pfu_biu_req_hit_idx = inst_info.page_ca && rb_entry_vld && !rb_entry_sync_fence &&
    (inst_info.addr(13,6) === rb_entry_cmp_pfu_biu_req_addr(13,6))

  //==========================================================
  //                 Flush dest_vld/Pop signal
  //==========================================================
  //req_biu_success && !cmit will cancel dest_vld
  rb_entry_flush_clear :=
    io.in.fromRTU.yy_xx_flush && (!rb_entry_cmit || inst_info.boundary && !rb_entry_secd) && !inst_info.mcic_req ||
    io.in.fromRTU.lsu_async_flush && !inst_info.mcic_req
  //==========================================================
  //              Interface to other module
  //==========================================================
  //-----------------cmit data vld signal---------------------
  //for rtu flush
  val rb_entry_cmit_data_not_vld = rb_entry_vld && rb_entry_cmit && rb_entry_dest_vld

  //==========================================================
  //                 Generate interface
  //==========================================================
  //------------------input-----------------------------------
  //-----------create signal--------------
  rb_entry_ld_create_vld        := io.in.RbEntry.ld_create_vld
  rb_entry_ld_create_dp_vld     := io.in.RbEntry.ld_create_dp_vld
  rb_entry_st_create_vld        := io.in.RbEntry.st_create_vld
  rb_entry_st_create_dp_vld     := io.in.RbEntry.st_create_dp_vld
  rb_entry_ld_create_gateclk_en := io.in.RbEntry.ld_create_gateclk_en
  rb_entry_st_create_gateclk_en := io.in.RbEntry.st_create_gateclk_en
  //-----------grnt signal----------------
  rb_entry_biu_pe_req_grnt      := io.in.RbEntry.biu_pe_req_grnt
  rb_entry_next_nc_bypass       := io.in.RbEntry.next_nc_bypass
  rb_entry_next_so_bypass       := io.in.RbEntry.next_so_bypass
  rb_entry_read_req_grnt        := io.in.RbEntry.read_req_grnt
  rb_entry_wb_cmplt_grnt        := io.in.RbEntry.wb_cmplt_grnt
  rb_entry_wb_data_grnt         := io.in.RbEntry.wb_data_grnt
  //----------gateclk signal--------------
  rb_entry_biu_id_gateclk_en    := io.in.RbEntry.biu_id_gateclk_en
  //------------------output----------------------------------
  //-----------rb entry signal------------
  io.out.state                 := rb_entry_state
  io.out.vld                   := rb_entry_vld
  io.out.mcic_req              := inst_info.mcic_req
  io.out.addr                  := inst_info.addr
  io.out.iid                   := inst_info.iid
  io.out.sign_extend           := inst_info.sign_extend
  io.out.preg                  := inst_info.preg
  io.out.page_ca               := inst_info.page_ca
  io.out.page_so               := inst_info.page_so
  io.out.page_sec              := inst_info.page_sec
  io.out.page_buf              := inst_info.page_buf
  io.out.page_share            := inst_info.page_share
  io.out.sync                  := inst_info.sync
  io.out.fence                 := inst_info.fence
  io.out.sync_fence            := rb_entry_sync_fence
  io.out.atomic                := inst_info.atomic
  io.out.ldamo                 := inst_info.ldamo
  io.out.st                    := inst_info.st
  io.out.bkpta_data            := inst_info.bkpta_data
  io.out.bkptb_data            := inst_info.bkptb_data
  io.out.inst_size             := inst_info.inst_size
  io.out.depd                  := rb_entry_depd
  io.out.vreg                  := inst_info.vreg
  io.out.inst_vfls             := inst_info.inst_vfls
  io.out.vreg_sign_sel         := inst_info.vreg_sign_sel
  io.out.priv_mode             := inst_info.priv_mode
  io.out.data                  := rb_entry_data
  io.out.bus_err               := rb_entry_bus_err
  io.out.flush_clear           := rb_entry_flush_clear
  io.out.cmit_data_vld         := !rb_entry_cmit_data_not_vld
  //-----------request--------------------
  io.out.biu_req               := rb_entry_biu_req
  io.out.create_lfb            := inst_info.create_lfb
  io.out.wb_cmplt_req          := rb_entry_wb_cmplt_req
  io.out.wb_data_req           := rb_entry_wb_data_req
  io.out.boundary_wakeup       := rb_entry_boundary_wakeup
  io.out.merge_fail            := rb_entry_merge_fail
  io.out.biu_pe_req            := rb_entry_biu_pe_req
  io.out.biu_pe_req_gateclk_en := rb_entry_biu_pe_req_gateclk_en
  //-----------barrier inst---------------
  io.out.fence_ld_vld          := rb_entry_fence_ld_vld
  //-----------hit idx--------------------
  io.out.ld_da_hit_idx         := rb_entry_ld_da_hit_idx
  io.out.st_da_hit_idx         := rb_entry_st_da_hit_idx
  io.out.sq_pop_hit_idx        := rb_entry_sq_pop_hit_idx
  io.out.wmb_ce_hit_idx        := rb_entry_wmb_ce_hit_idx
  io.out.pfu_biu_req_hit_idx   := rb_entry_pfu_biu_req_hit_idx
  //-----------other signal---------------
  io.out.discard_vld           := rb_entry_discard_vld
  io.out.rot_sel               := rb_entry_rot_sel
  io.out.wb_data_pre_sel       := rb_entry_wb_data_pre_sel
  io.out.atomic_next_resp      := rb_entry_atomic_next_resp

}
