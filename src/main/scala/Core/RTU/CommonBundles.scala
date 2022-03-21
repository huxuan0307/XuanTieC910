package Core.RTU

import Core.AddrConfig.PcWidth
import Core.ExceptionConfig._
import chisel3._
import chisel3.util._
import Core.IntConfig._
import Core.ROBConfig._
import Core.VectorUnitConfig._

class RegEntryFromCp0Bundle extends Bundle {
  val rtuIcgEn : Bool = Bool()
  val yyClkEn : Bool = Bool()
}

class PregFromRetire(width: Int) extends Bundle {
  val asyncFlush : Bool = Bool()
  val wbRetireInstPregValid : Vec[Bool] = Vec(width, Bool())
}

class RobFromCp0Bundle extends Bundle {
  val icgEn : Bool = Bool()
  val xxIntB : Bool = Bool()
  val xxVec : UInt = UInt(InterruptVecWidth.W)
  val yyClkEn : Bool = Bool()
}

class RobFromHadBundle extends Bundle {
  val dataBreakPointDebugReq : Bool = Bool()
  val debugReqEn : Bool = Bool()
  val debugRetireInfoEn : Bool = Bool()
  val instBreakPointDebugReq : Bool = Bool()
  val xxTme : Bool = Bool()
}

class RtuFromHpcpBundle extends Bundle {
  val cntEn : Bool = Bool()
}

class RobFromIfu extends Bundle {
  val curPc : UInt = UInt(PcWidth.W)
  val curPcLoad : Bool = Bool()
}

class RtuFromPadBundle extends Bundle {
  val yyIcgScanEn : Bool = Bool()
}

abstract class RobFromRetire extends Bundle {
  // Todo: check if need to rename -> interruptMask
  val asyncExceptionCommitMask : Bool = Bool()
  val ctcFLushReq : Bool = Bool()
  val debug = new Bundle() {
    val instAckInt : Bool = Bool()
    val debugModeOn : Bool = Bool()
    val exceptionValid : Bool = Bool()
    val instflush : Bool = Bool()
    val mispred : Bool = Bool()
  }
  val flush : Bool = Bool()
  val flushState : UInt = UInt(FlushState.width.W)
  val flushGateClk : Bool = Bool()
  val instJmp : Vec[Bool] = Vec(3, Bool())
  val instFlush : Bool = Bool()
  val rtMask : Bool = Bool()
  // flush of vector first only fault instruction
  val splitFofFlush : Bool = Bool()
  val srtEn : Bool = Bool()
  val retireEmpty : Bool = Bool()
}

class RobFromVfpu extends Bundle {
  val pipe6 : RobFromPipeCommonBundle = new RobFromPipeCommonBundle {}
  val pipe7 : RobFromPipeCommonBundle = new RobFromPipeCommonBundle {}
}

class RobCreateBundle extends Bundle {
  val data = new RobEntryData
  val dpEn : Bool = Bool()
  val en : Bool = Bool()
  val gateClkEn : Bool = Bool()
}

abstract class RobFromPipeCommonBundle extends Bundle {
  val iid : UInt = UInt(InstructionIdWidth.W)
  val cmplt : Bool = Bool()
}

abstract class RobFromLsuPipeCommonBundle extends Bundle {
  val splitSpecFailIid = ValidIO(UInt(InstructionIdWidth.W))
  val iid : UInt = UInt(InstructionIdWidth.W)
  val cmplt : Bool = Bool()
  val abnormal : Bool = Bool()
  val breakpointData = new RobBreakpointDataBundle
  val exceptVec = ValidIO(UInt(ExceptionVecWidth.W))
  val flush : Bool = Bool()
  val mtval : UInt = UInt(MtvalWidth.W)
  val noSpec = new RobNoSpecBundle
  val specFail : Bool = Bool()
  val vstart = ValidIO(UInt(VlmaxBits.W))
  val vsetvl : Bool = Bool()
}

class RobToRetireInstBundle extends Bundle {
  val valid : Bool = Bool()
  val bju : Bool = Bool()
  val condBranch : Bool = Bool()
  val condBrTaken : Bool = Bool()
  val jmp : Bool = Bool()
  val load : Bool = Bool()
  val split : Bool = Bool()
  val store : Bool = Bool()
  val vlPred : Bool = Bool()
  val vsetvli : Bool = Bool()
  val fpDirty : Bool = Bool()
  val vecDirty : Bool = Bool()
  val noSpec = new RobNoSpecBundle
  val pstEregValid : Bool = Bool()
  val pstPregValid : Bool = Bool()
  val pstVregValid : Bool = Bool()
  // Todo: imm, figure out
  val checkIdx : UInt = UInt(8.W)
  val pc : UInt = UInt(PcWidth.W)
  val npc : UInt = UInt(PcWidth.W)
  // Todo: imm
  val pcOffset : UInt = UInt(RobPcOffsetBits.W)
  // Todo: imm
  val num : UInt = UInt(2.W)
  val vl : UInt = UInt(VlmaxBits.W)
  val vlmul : UInt = UInt(VlmulBits.W)
  val vsew : UInt = UInt(VsewBits.W)
}

class RobToRetireInstExtraBundle extends Bundle {
  val bhtMispred : Bool = Bool()
  val bjuIncPc : UInt = UInt(PcWidth.W)
  val breakPoint : Bool = Bool()
  val ctcFlush : Bool = Bool()
  val dataBreakpoint : Bool = Bool()
  val debugDisable : Bool = Bool()
  val efPcValid : Bool = Bool()
  // Todo: imm
  val exceptionVec = ValidIO(UInt(4.W))
  val highHwException : Bool = Bool()
  val iid : UInt = UInt(InstructionIdWidth.W)
  val instMmuException : Bool = Bool()
  val instBreakPoint : Bool = Bool()
  val instFlush : Bool = Bool()
  val interruptVec = ValidIO(UInt(InterruptVecWidth.W))
  val interruptMask : Bool = Bool()
  val jmpMispred : Bool = Bool()
  val mtval : UInt = UInt(MtvalWidth.W)

  val pCall : Bool = Bool()
  val pReturn : Bool = Bool()
  val ras : Bool = Bool()
  val specFail : Bool = Bool()
  val specFailNoSsf : Bool = Bool()
  val specFailSsf : Bool = Bool()
  val vsetvl : Bool = Bool()
  val vstart = ValidIO(UInt(VlmaxBits.W))
}

abstract class RobToRetireBundle extends Bundle {
  // Todo: imm
  val commitValidVec : Vec[Bool] = Vec(NumCommitEntry, Bool())
  val ctcFlushSrtEn : Bool = Bool()
  val instVec = Vec(NumRetireEntry, new RobToRetireInstBundle)
  val instExtra = new RobToRetireInstExtraBundle
  val robCurPc : UInt = UInt(PcWidth.W)
  val splitSpecFailSrt : Bool = Bool()
  val intSrtEn : Bool = Bool()
  val ssfIid : UInt = UInt(InstructionIdWidth.W)
}

class RobToCpu extends Bundle {
  val noRetire : Bool = Bool()
}

class RobToHad extends Bundle {
  val breakpointDataSt : Bool = Bool()
  val dataBreakpoint = new RobBreakpointDataBundle
  val instBreakpoint = new RobBreakpointInstBundle
  val instBreakpointInstValid : Bool = Bool()
  val instExeDead : Bool = Bool()
  val instSplit : Bool = Bool()
  // Todo: imm
  val instNonIrvBreakpoint : Vec[RobBreakpointBundle] = Vec(3, new RobBreakpointBundle)
  val retireInstInfo : Vec[ValidIO[UInt]] = Vec(NumRetireEntry, ValidIO(UInt(NumRobEntry.W)))
  val robEmpty : Bool = Bool()
  val xxMBreakpointChangeFlow : Bool = Bool()
}

class RobToHpcp extends Bundle {
  // Todo: imm
  val instCurPc : Vec[UInt] = Vec(3, UInt(PcWidth.W))
  val instJmpPcOffset8m : Vec[Bool] = Vec(3, Bool())
}

class RobToIu extends Bundle {
  // Todo: check
  val robReadPcFifoValid : Vec[Bool] = Vec(NumRobReadEntry, Bool())
  val robReadPcFifoGateClkValid : Bool = Bool()
}

class RobToPad extends Bundle {
  // Todo: Figure out why width(retirePc) = PcWidth+1
  val retirePc : Vec[ValidIO[UInt]] = Vec(NumRetireEntry, ValidIO(UInt((PcWidth+1).W)))
}

class RobYyXx extends Bundle {
  // Todo: check
  val commitIid : Vec[ValidIO[UInt]] = Vec(NumCommitEntry, ValidIO(UInt(InstructionIdWidth.W)))
  val retire : Vec[Bool] = Vec(NumRetireEntry, Bool())
}

class RegEntryFromIfuBundle extends Bundle {
  val xxSyncReset : Bool = Bool()
}

class RobNoSpecBundle extends Bundle {
  val mispred     : Bool = Bool()
  val miss        : Bool = Bool()
  val hit         : Bool = Bool()
}

class RobBreakpointDataBundle extends Bundle {
  val a           : Bool = Bool()
  val b           : Bool = Bool()
}

class RobBreakpointInstBundle extends Bundle {
  val a           : Bool = Bool()
  val b           : Bool = Bool()
}

class RobBreakpointBundle extends Bundle {
  val inst = new RobBreakpointInstBundle
  val data = new RobBreakpointDataBundle
}

class PstPregEntryCreateBundle extends Bundle {
  val pregValid : Bool = Bool()
  val dstReg : UInt = UInt(NumLogicRegsBits.W)
  val preg : UInt = UInt(NumPhysicRegsBits.W)
  val relPreg : UInt = UInt(NumPhysicRegsBits.W)
  val iid : UInt = UInt(InstructionIdWidth.W)
}

class PstVFregEntryCreateBundle extends Bundle {
  val vregValid : Bool = Bool()
  val fregValid : Bool = Bool()
  val dstReg : UInt = UInt(NumLogicRegsBits.W)
  val preg : UInt = UInt(NumVPregsBits.W)
  val relPreg : UInt = UInt(NumVPregsBits.W)
  val iid : UInt = UInt(InstructionIdWidth.W)
}

class PstEregEntryCreateBundle extends Bundle {
  val eregValid = Bool()
  val ereg = UInt(NumEregsBits.W)
  val relEreg = UInt(NumEregsBits.W)
  val iid : UInt = UInt(InstructionIdWidth.W)
}