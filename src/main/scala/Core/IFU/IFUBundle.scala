package Core.IFU
//import Core.IDU.IDData
import Core.AddrConfig.PcWidth
import Core.RTU._
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._

class IDData extends  Bundle {
  val vl_pred      = Bool()//72
  val vl           = UInt(8.W)
  val pc           = UInt(15.W)
  val vsew         = UInt(3.W)
  val vlmul        = UInt(2.W)
  val no_spec      = Bool()//43
  val bkptb_inst   = Bool()
  val bkpta_inst   = Bool()
  val split_short  = Bool()
  val fence        = Bool()
  val split_long   = Bool()
  val high_hw_expt = Bool()//37
  val expt_vec     = UInt(4.W)
  val expt_vld     = Bool()//32
  val opcode       = UInt(32.W)//31
}

class PCGenIO extends CoreBundle{
  //0 IFStage, 1 IPStage, 2 IBStage, 3 BRU
  val redirect = Vec(4,Flipped(Valid(UInt(VAddrBits.W))))
  val pc = Output(UInt(VAddrBits.W))
  val continue = Input(Bool())
  val IbufAllowEnq = Input(Bool())
  val had_ifu_pc = Input(UInt(VAddrBits.W))
  val had_ifu_pcload = Input(Bool())
  val vector_pcgen_pc = Input(UInt(VAddrBits.W))
  val vector_pcgen_pcload = Input(Bool())
  //val vector_pcgen_reset_on = Input(Bool())
  //val rtu_ifu_chgflw_pc = Input(UInt(VAddrBits.W))
  //val rtu_ifu_chgflw_vld = Input(Bool())
  //val rtu_ifu_xx_dbgon = Input(Bool())
  //val rtu_ifu_xx_expt_vld = Input(Bool())
  val ifu_rtu_cur_pc = Output(UInt(VAddrBits.W))
  val ifu_rtu_cur_pc_load = Output(Bool())
}

class BHT_IP_Resp extends CoreBundle {
  val pre_taken  = Vec(16,UInt(2.W))
  val pre_ntaken = Vec(16,UInt(2.W))
  val pre_offset = UInt(4.W)
  val pre_sel    = UInt(2.W)
}

class ICacheReq extends CoreBundle {
  val vaddr = UInt(VAddrBits.W)
  val paddr = UInt(PAddrBits.W)
}

class ICacheResp extends CoreBundle {
  val inst_data = Vec(8,UInt(16.W))   //h0 +8
  val predecode = Vec(8,UInt(4.W))
}

class IFU_TLB extends CoreBundle {
  val vaddr = ValidIO(UInt(VAddrBits.W))
  val paddr = Flipped(ValidIO(UInt(PAddrBits.W)))
  val tlb_miss = Input(Bool())
}

class IP2IB extends CoreBundle {
  val pc = UInt(VAddrBits.W)
  val icache_resp = new ICacheResp
  val decode_info = new IPDecodeOutput

  val bht_resp  = new BHT_IP_Resp
  val bht_res = UInt(2.W)
  val is_ab_br = Bool()
  val br_position = UInt(4.W)
  val br_offset = UInt(21.W)
  val br_valid = Bool()
  val btb_valid = Bool()
  val btb_target = UInt(20.W)
  val btb_miss = Bool()
  val btb_sel  = UInt(4.W)
  val ubtb_valid = Bool()
  val ubtb_resp = new uBTBResp
  val ubtb_miss = Bool()
  val ubtb_mispred = Bool()
  val pcall = Bool()
  val pret  = Bool()
  val ind_vld = Bool()
  val push_pc = UInt(VAddrBits.W)
  val h0_vld = Bool()
  val h0_data = UInt(8.W)
  val h0_predecode = UInt(4.W)
  val inst_32_9 = UInt(9.W)
  val chgflw_vld_mask = UInt(9.W)
  val h0_pc     = UInt(VAddrBits.W)
  val cur_pc    = Vec(8,UInt(VAddrBits.W))
  val jal       = UInt(8.W)
  val jalr       = UInt(8.W)
  val con_br       = UInt(8.W)
  val dst       = UInt(8.W)
}

class IPStageIO extends  CoreBundle {
  val if_vld = Input(Bool())
  val pc = Input(UInt(VAddrBits.W))
  val ip_redirect = Valid(UInt(VAddrBits.W))
  val ip_flush = Input(Bool())

  val ubtb_resp   = Flipped(Valid(new uBTBResp))  //RegNext
  val icache_resp = Flipped(Valid(new ICacheResp))
  val bht_resp    = Input(new BHT_IP_Resp)
  val ip_bht_con_br_taken = Output(Bool())
  val ip_bht_con_br_vld   = Output(Bool())
  val btb_resp    = Vec(4,Flipped(Valid(UInt(20.W))))


  val out = Valid(new IP2IB)
}

class IBStageIO extends CoreBundle {
  val ip2ib       = Flipped(Valid(new IP2IB))

  val ib_redirect = Valid(UInt(VAddrBits.W))

  val ind_jmp_valid  = Output(Bool())
  val ind_btb_target = Input(UInt(20.W))
  val ras_target_pc  = Input(UInt(VAddrBits.W))  //ras stack top

  val btb_update       = Valid(new BTBUpdate)
  val ubtb_update_data = Valid(new uBTBUpdateData)
  val ubtb_update_idx  = Valid(UInt(16.W))
}


class BPUUpdate extends CoreBundle {
  val rtu_flush = Input(Bool())

  val rtu_retire_condbr       = Input(Vec(3,Bool()))
  val rtu_retire_condbr_taken = Input(Vec(3,Bool()))
  val bht_update = Flipped(Valid(new BHTUpdate))//cur_condbr_taken from BJU

  val rtu_ras_update = new RASUpdateIO

  val ind_btb_commit_jmp_path = Flipped(Vec(3,Valid(UInt(8.W))))//valid排序依次进入
  val ind_btb_rtu_jmp_mispred = Input(Bool())
  val ind_btb_rtu_jmp_pc      = Input(UInt(VAddrBits.W))//pc(21,1)
}
class RobFromIfu extends Bundle {
  val curPc : UInt = UInt(PcWidth.W)
  val curPcLoad : Bool = Bool()
}

class PCfifoIO extends Bundle {
  val in = Input(new PCfifo_iu)
  val out = Output(Vec(2, new BhtPredDataForward))
}
class PCfifo_iu extends Bundle with Config {
  val target_pc = UInt(PcWidth.W)
  val h0_vld    = Bool()
  val cur_pc    = Vec(9, UInt(PcWidth.W))
  val pc_oper   = UInt(8.W)
  val jal       = UInt(8.W)
  val jalr      = UInt(8.W)
  val con_br    = UInt(8.W)
  val dst_vld   = UInt(8.W)
  val sel_res   = UInt(2.W)
  val pre_res   = UInt(2.W)
  val vghr      = UInt(ghr_size.W)
  val ind_btb_miss = Bool()
}
class BhtPredDataForward extends Bundle{ // @ct_iu_bju_pcfifo @1677-1684
  var curPc = UInt(PcWidth.W)
  var tarPc = UInt(PcWidth.W)
  var jal = Bool()
  var jalr = Bool()
  var dstVld = Bool()
  val predStore = new IfuPredStore
}
class IfuPredStore extends Bundle{
//  val pc = UInt(PcWidth.W)
  val bhtPred = Bool()
  val chkIdx = UInt(25.W)
  val jmpMispred = Bool()
}
class IFUIO extends CoreBundle {
  //flush
  val bru_redirect = Flipped(Valid(UInt(VAddrBits.W)))
  //inst fetch
  val tlb        = new IFU_TLB
  //val cache_req  = Decoupled(new ICacheReq)
  //val cache_resp = Flipped(Valid(new ICacheResp))
  //inst out
  val instData = Output(Vec(3, new IDData))
  val instVld  = Output(Vec(3, Bool()))
  //bht, btb update
  val bpu_update = new BPUUpdate
  val pc = Output(UInt(VAddrBits.W))
  val toROB = Output(new RobFromIfu)
  val ifuForward = Output(Vec(2,new BhtPredDataForward))
}