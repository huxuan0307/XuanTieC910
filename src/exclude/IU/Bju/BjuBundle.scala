package Core.IU.Bju

import Core.AddrConfig.PcWidth
import chisel3._

class BhtPredDataForward extends Bundle{ // @ct_iu_bju_pcfifo @1677-1684
  var curPc = UInt(PcWidth.W)
  var tarPc = UInt(PcWidth.W)
  var jal = Bool()
  var jalr = Bool()
  var dstVld = Bool()
  val predStore = new IfuPredStore
}

class IfuPredStore extends Bundle{
  val pc = UInt(PcWidth.W)
  val bhtPred = Bool()
  val chkIdx = UInt(25.W)
  val jmpMispred = Bool()
}

class PredCheckRes extends Bundle{
  val length = Bool()
  val bhtMispred = Bool()
  val jmp = Bool()
  val condbr = Bool()
  val pcall = Bool()
  val pret = Bool()
  val pc = UInt(PcWidth.W)
}

