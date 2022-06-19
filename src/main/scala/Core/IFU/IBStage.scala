package Core.IFU
import Utils.{SignExt, UIntToMask}
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._

class IBStage extends Module with Config {
  val io = IO(new IBStageIO)

  val ind_btb_rd_state = RegInit(false.B)
  val pc_oper_updt_vldReg = RegInit(false.B)
  val hn_pc_oper_updt_valReg = RegInit(0.U(8.W))
  val ibdp_pcfifo_if_hn_pc_oper = WireInit(0.U(8.W))
  val ibctrl_ibdp_cancel = WireInit(false.B)
  val pc_oper_updt_vld_pre = WireInit(false.B)

  val ib_cancel = io.pcgen_ibctrl_cancel

  val ib_data_valid = io.ip2ib.valid

  //check ubtb btb
  val ubtb_br_miss = io.ip2ib.bits.ubtb_miss
  val ubtb_br_mispred = io.ip2ib.bits.ubtb_mispred

  val ubtb_ras_pc_hit = io.ras_target_pc === io.ip2ib.bits.ubtb_resp.target_pc
  val ubtb_ras_mistaken = io.ip2ib.bits.ubtb_valid && io.ip2ib.bits.ubtb_resp.is_ret && !io.ip2ib.bits.pret   //is not return,but ubtb predict it is return,redirect in ip stage
  val ubtb_ras_miss     = (!io.ip2ib.bits.ubtb_valid || !io.ip2ib.bits.ubtb_resp.is_ret) && io.ip2ib.bits.pret
  val ubtb_ras_mispred  = io.ip2ib.bits.ubtb_valid && io.ip2ib.bits.ubtb_resp.is_ret && io.ip2ib.bits.pret && !ubtb_ras_pc_hit //is return,ubtb is return
  val ubtb_ras_hit      = io.ip2ib.bits.ubtb_valid && io.ip2ib.bits.ubtb_resp.is_ret && io.ip2ib.bits.pret && ubtb_ras_pc_hit

  val br_target1_8 = Cat(io.ip2ib.bits.pc(VAddrBits-1, 4), 0.U(4.W)) + SignExt(io.ip2ib.bits.br_offset,VAddrBits) + (io.ip2ib.bits.br_position << 1.U)
  val br_target_0  = Cat(io.ip2ib.bits.pc(VAddrBits-1, 4), 0.U(4.W)) + SignExt(io.ip2ib.bits.br_offset,VAddrBits) - 2.U
  val br_target    = Mux(io.ip2ib.bits.h0_vld && io.ip2ib.bits.br_position === 0.U, br_target_0, br_target1_8)
  val btb_miss    = io.ip2ib.bits.btb_miss
  val btb_mispred = io.ip2ib.bits.br_valid && io.ip2ib.bits.btb_valid && br_target(20,1) =/= io.ip2ib.bits.btb_target



  //redirect         icache predecode   ip predecode  ip predecode      ip predecode      ip predecode
  val chgflw_vld = ib_data_valid && (btb_miss || btb_mispred || ubtb_ras_mispred || ubtb_ras_miss || io.ip2ib.bits.ind_vld)
  io.ib_redirect.valid := chgflw_vld
  val ind_btb_pc = Cat(io.ip2ib.bits.pc(VAddrBits-1,21),io.ind_btb_target,0.U(1.W))
  //val br_target  = Cat(io.ip2ib.bits.pc(VAddrBits-1,21),io.ip2ib.bits.br_offset)
  io.ib_redirect.bits  := Mux(io.ip2ib.bits.ind_vld, ind_btb_pc, Mux(ubtb_ras_mispred || ubtb_ras_miss, io.ras_target_pc, br_target))
  //io.ib_redirect_pcload_vld := chgflw_vld && !ib_chgflw_self_stall todo: pcgen_l0_btb_chgflw_vld

  //btb update
  io.btb_update.valid := (btb_miss || btb_mispred) && ib_data_valid
  io.btb_update.bits.btb_data  := br_target(20,1)
  io.btb_update.bits.btb_index := io.ip2ib.bits.pc(13,4)
  io.btb_update.bits.btb_tag   := Cat(io.ip2ib.bits.pc(20,14), io.ip2ib.bits.pc(3,1))

  //ubtb update
  val bht_res = io.ip2ib.bits.bht_res
  val ubtb_br_miss_update = ubtb_br_miss && !btb_miss && (io.ip2ib.bits.is_ab_br || bht_res === 3.U)
  io.ubtb_update_data.valid  := ib_data_valid && (ubtb_br_miss_update || ubtb_br_mispred || ubtb_ras_mistaken || ubtb_ras_miss || ubtb_ras_mispred)
  io.ubtb_update_data.bits.tag := io.ip2ib.bits.pc(15,1)
  io.ubtb_update_data.bits.target := io.ip2ib.bits.btb_target
  io.ubtb_update_data.bits.ras := io.ip2ib.bits.pret
  io.ubtb_update_data.bits.entry_valid := !ubtb_ras_mistaken
  io.ubtb_update_idx.valid   := ib_data_valid && (ubtb_br_mispred || ubtb_ras_mistaken || ubtb_ras_mispred)
  io.ubtb_update_idx.bits    := io.ip2ib.bits.ubtb_resp.hit_index

  io.ind_jmp_valid := ib_data_valid && io.ip2ib.bits.ind_vld   //todo

  //////todo: ib_expt_vld comes from ip/if, concerning refill and mmu
  val ib_expt_vld = false.B
  //////todo: addrgen cancel signal
  val ib_addr_cancel = false.B

  //ib_chgflw_self_stall
  //  1.ibuf full or lbuf special state stall
  //  2.more than 2 pc_oper/load inst stall
  //  3.pcfifo full stall
  //  4.iu mispredict valid & ifu fetch indbr/ras inst stall
  //  5.ind_btb result not valid will satll one cycle
  //For 2, fifo_stall & 5, ind_btb_rd_stall:
  //if it not caccel ib_chgflw
  //next cycle pipeline will stall until ib chgflw happen
  //ib change flow will cancel previous ib chgflw
  //Thus we need not use fifo stall cancel ib chgflw
  //  val ibctrl_ibdp_mispred_stall    = mispred_stall
  //  val ibctrl_ibdp_fifo_stall       = fifo_stall
  //  val ibctrl_ibdp_buf_stall        = buf_stall
  //  val ibctrl_ibdp_fifo_full_stall  = fifo_full_stall
  //  val ibctrl_ibdp_ind_btb_rd_stall = ind_btb_rd_stall
  //==========================================================
  //                   IB Stage Stall
  //==========================================================
  //---------------------Take Notice--------------------------
  //idu_ifu_id_bypass_stall > idu_stall > iu_ifu_mispred_stall
  //Which can be used to simplify logic

  //ibctrl_ipctrl_stall

  //-----------------------buf_stall--------------------------
  //buf_stall valid when
  //  1.ibuf full or lbuf special state/full
  //  2.ip_ib_vld
  val buf_stall  = ib_data_valid && io.ibuf_ibctrl_stall //ibuf_full
   //|| lbuf_ibctrl_stall //lbuf front_branch/cache state todo: add lubf

  //--------------------fifo_full_stall----------------------
  //lbuf active state, pcfifo will also be full
  //Thus lbuf active should be treated as ib_data_vld
  val fifo_full_stall  = io.iu_ifu_pcfifo_full &&
    (ib_data_valid) //todo: add lbuf
  //----------------------mispred_stall----------------------
  //mispred_stall valid when
  //  1.iu mispred stall valid(from next cycle of mispred)
  //  2.ifu ib stage fetch ras/ind_br inst
  //  3.in case of IND_BTB or RAS predict Mistake
  //mispred stall used for maintain ras & ind_btb right
  //if iu_ifu_mispred_stall signal need ifu generate
  //when iu_mispred, mispred_state_reg <= 1
  //when rtu_retire_mispred, mispred_state_reg <= 0
  val hn_pcall = io.ip2ib.bits.hn_pcall & Cat(Seq.fill(8)(~ib_expt_vld))
  val hn_preturn = io.ip2ib.bits.hn_pret & Cat(Seq.fill(8)(~ib_expt_vld))
  val hn_ind_br = io.ip2ib.bits.hn_ind_br & Cat(Seq.fill(8)(~ib_expt_vld))
  val mispred_stall   = io.iu_ifu_mispred_stall &&
    ib_data_valid &&
    !ib_expt_vld &&
    (
      (
        hn_pcall.orR ||
  hn_preturn.orR ||
  hn_ind_br.orR
  )
  )

  //if RAS not valid
  //take next 128 PC as predict PC
  val ras_chgflw_vld  = ib_data_valid && (hn_preturn.orR)
  //////todo: add signals
  val ib_chgflw_self_stall = buf_stall ||
    fifo_full_stall ||
    mispred_stall
  //-----------------------idu_stall--------------------------
  //idu_stall means idu_ifu_id_stall
  //which stop inst trans from ifu to idu
  val idu_stall       = io.idu_ifu_id_stall

  //----------------------fifo_stall--------------------------
  //fifo_stall get may use 7-8 Gate,
  //which will make ib_chgflw_vld timing worse
  //Considering ib chgflw not use fifo_stall
  //And correct it by cancel IF stage
  //fifo_stall valid when
  //  1.pcfifo more than two inst
  //  2.ip_ib_valid
  val pcfifo_stall    = io.pcfifo_if_ibctrl_more_than_two && !ib_expt_vld
  val fifo_stall      = ib_data_valid && pcfifo_stall

  //------------------ind_btb_rd_stall-----------------------
  //ind_btb_rd_stall happens at the moment
  //ib stage fetch ind_btb inst && ind_btb jmp state = 0
  //Because ind_btb is rd in IB Stage,
  //whose result will not be got cur cycle
  //Thus we use ind_btb_rd_stall to stall one cycle
  //and wait ind_btb result
  val ind_btb_rd_stall = !ind_btb_rd_state &&
    ib_data_valid &&
    !ib_expt_vld &&
    (hn_ind_br.orR)

  val ind_jmp_success = ind_btb_rd_state && io.ib_redirect.valid && !fifo_stall && !fifo_full_stall ////todo:replace ib_redirect.valid with chgflw_vld
  ind_btb_rd_state := PriorityMux(Seq(
    ib_cancel -> false.B,
    ind_btb_rd_stall -> true.B,
    ind_jmp_success -> false.B,
    true.B -> ind_btb_rd_state
  ))

  io.ibctrl_ipctrl_stall := mispred_stall ||
    buf_stall ||
    fifo_full_stall ||
    fifo_stall ||
    ind_btb_rd_stall
  //==========================================================
  //                   IB Stage Cancel
  //==========================================================
  //----------------------ib_cancel---------------------------
  //ib_cancel means higher priority unit chgflw
  //which should cancel ib stage operation:
  //  1.cancel ibuf/lbuf write in
  //  2.cancel pcfifo/ldfifo write in
  //  3.cancel ibuf inst send to idu
  //  4.cancel lbuf inst send to idu
  //  5.cancel bypass inst send to idu
  //==========================================================
  //                  pcfifo control signal
  //==========================================================
  //pcfifo full will be maintained by iu

  val pcfifo_create_vld = ib_data_valid &&
    !ib_cancel &&
    !ib_expt_vld &&
    //!ib_addr_cancel && todo
    !mispred_stall &&
    !ind_btb_rd_stall &&
    !buf_stall && !fifo_full_stall
  io.fifo_create_vld := pcfifo_create_vld
  io.ibctrl_pcfifo_if_ras_vld := ras_chgflw_vld
  io.ibctrl_pcfifo_if_ras_target_pc := io.ras_target_pc
  io.ibctrl_pcfifo_if_ind_target_pc := ind_btb_pc //////todo: complete it
  io.ibdp_pcfifo_if_ind_br_offset := io.ip2ib.bits.br_offset //has been check

  ibctrl_ibdp_cancel := ib_addr_cancel || ib_cancel
  //==========================================================
  //               hn_pc_oper pcfifo mask
  //==========================================================
  pc_oper_updt_vld_pre := ib_data_valid && !ind_btb_rd_stall &&
    (
        mispred_stall ||
        fifo_full_stall ||
        fifo_stall ||
        buf_stall
    )
  pc_oper_updt_vldReg := Mux(ibctrl_ibdp_cancel,false.B,pc_oper_updt_vld_pre)
  ibdp_pcfifo_if_hn_pc_oper := Mux(pc_oper_updt_vldReg,hn_pc_oper_updt_valReg,io.ip2ib.bits.pc_oper)
  hn_pc_oper_updt_valReg := Mux(mispred_stall || io.iu_ifu_pcfifo_full || buf_stall,ibdp_pcfifo_if_hn_pc_oper, ibdp_pcfifo_if_hn_pc_oper & io.pc_oper_over_mask)
  io.ibdp_pcfifo_if_hn_pc_oper := ibdp_pcfifo_if_hn_pc_oper
  //  io.ibctrl_mispred_stall := mispred_stall
  //  io.ibctrl_fifo_stall := fifo_stall
  //  io.ibctrl_buf_stall := buf_stall
  //  io.ibctrl_fifo_full_stall := fifo_full_stall
  //  io.ibctrl_ind_btb_rd_stall := ind_btb_rd_stall


}
