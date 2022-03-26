package Core.IS

import Core.ExceptionConfig._
import Core.IntConfig._
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._

trait VfiqConfig {
  def NumSrcVf = 4
  def NumEntryVfiq = 8
}

object VfiqConfig extends VfiqConfig

class VfiqEntryData extends Bundle with VfiqConfig {
  val vl = UInt(VlmaxBits.W)
  val vsew = UInt(VsewBits.W)
  val vlmul = UInt(VlmulBits.W)
  val vmul = Bool()
  val vmulUnsplit = Bool()
  val vmlaShort = Bool()  // 136
  val vdiv = Bool()       // 135
  val launchReadyViq1 = Vec(NumEntryVfiq, Bool()) // 134
  val launchReadyViq0 = Vec(NumEntryVfiq, Bool()) // 126
  // Todo: imm
  val vmlaType = UInt(3.W) // 118
  val splitNum = UInt(7.W) // 115
  val splitLast = Bool() // 108
  val mfvr = Bool() // 107
  val vmla = Bool() // 106
  // Todo: Replace with DepVregEntryData
  val srcVec = Vec(NumSrcVf, new DepRegEntryData) // 105 ~ 65
  val dstEreg = UInt(NumEregsBits.W) // 64
  val dstVreg = UInt(NumVPregsBits.W) // 59
  val dstPreg = UInt(NumPhysicRegsBits.W) // 52
  val dstEregValid = Bool()
  val dstVregValid = Bool()
  val dstPregValid = Bool()
  val srcValidVec = Vec(NumSrcVf, Bool())
  val iid = UInt(InstructionIdWidth.W)
  val opcode = UInt(OpcodeBits.W)
}
