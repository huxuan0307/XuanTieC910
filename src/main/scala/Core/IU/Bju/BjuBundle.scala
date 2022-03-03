package Core.IU.Bju

import Core.AddrConfig.PcWidth
import chisel3._

class BhtPredDataForward extends Bundle{ // @ct_iu_bju_pcfifo @1677-1684
  var cur_pc = UInt(PcWidth.W)
  var tar_pc = UInt(PcWidth.W)
  var jal = Bool()
  var jalr = Bool()
  var dst_vld = Bool()
  val pred_store = new IfuPredStore
}

class IfuPredStore extends Bundle{
  val pc = UInt(PcWidth.W)
  val bht_pred = Bool()
  val chk_idx = UInt(25.W)
  val jmp_mispred = Bool()
}

class PredCheckRes extends Bundle{
  val length = Bool()
  val bht_mispred = Bool()
  val jmp = Bool()
  val condbr = Bool()
  val pcall = Bool()
  val pret = Bool()
  val pc = UInt(PcWidth.W)
}

