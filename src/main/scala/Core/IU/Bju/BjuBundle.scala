package Core.IU.Bju

import Core.AddrConfig.PcWidth
import chisel3._

class ifuDataForward extends Bundle{ // @ct_iu_bju_pcfifo @1677-1684
  var curPc = UInt((PcWidth).W) // TODO why 40 bits?
  var dstVld = Bool()
  val en = Bool()
  var tarPc = UInt((PcWidth).W)
  var jal = Bool()
  var jalr = Bool()
  val predStore = new IfuPredStore
}

class IfuPredStore extends Bundle{
  val bhtPred = Bool()
  val chkIdx = UInt(25.W)
  val jmpMispred = Bool()
  val pc = UInt((PcWidth).W) // pc ignore in forward, but use in store fifo
}

class PredCheckRes extends Bundle{
  val instVld = Bool()
  val bhtMispred = Bool()
  val jmp = Bool()
  val pcall = Bool()
  val pret = Bool()
  val pc = UInt((PcWidth).W)
  val condbr = Bool()
  val length = Bool()
  val bhtPred = Bool()
}

