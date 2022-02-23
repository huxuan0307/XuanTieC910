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
  // Todo: Imm
  val xxVec : UInt = UInt(5.W)
  val yyClkEn : Bool = Bool()
}

class RobFromHadBundle extends Bundle {
  val dataBreakPointDebugReq : Bool = Bool()
  val debugReqEn : Bool = Bool()
  val debugRetireInfoEn : Bool = Bool()
  val instBreakPointDebugReq : Bool = Bool()
  val xxTme : Bool = Bool()
}

class RobFromHpcpBundle extends Bundle {
  val cntEn : Bool = Bool()
}

class RobFromIfu extends Bundle {
  val curPc : UInt = UInt(PcWidth.W)
  val curPcLoad : Bool = Bool()
}

class RobFromPad extends Bundle {
  val yyIcgScanEn : Bool = Bool()
}

abstract class RobFromRetire extends Bundle {
  val asyncExceptionCommitMask : Bool = Bool()
  val ctcFLushReq : Bool = Bool()
  val debug = new Bundle() {
    val instAckInt : Bool = Bool()
    val debugModeOn : Bool = Bool()
    val exceptionValid : Bool = Bool()
    val flush : Bool = Bool()
    val mispred : Bool = Bool()
  }
  val flush : Bool = Bool()
  val flushGateClk : Bool = Bool()
  val instJmp : Vec[Bool] = Vec(3, Bool())
  val instFlush : Bool = Bool()
  val rtMask : Bool = Bool()
  val splitFofFlush : Bool = Bool()
  val srtEn : Bool = Bool()
}

class RobFromVfpu extends Bundle {
  val pipe6 : RobFromPipeCommonBundle = new RobFromPipeCommonBundle {}
  val pipe7 : RobFromPipeCommonBundle = new RobFromPipeCommonBundle {}
}

class RobCreateBundle extends Bundle {
  val data : UInt = UInt(40.W)
  val dpEn : Bool = Bool()
  val en : Bool = Bool()
  val gateClkEn : Bool = Bool()
}

abstract class RobFromPipeCommonBundle extends Bundle {
  val iid : UInt = UInt(InstructionIdWidth.W)
  val cmplt : Bool = Bool()
}

abstract class RobFromLsuPipeCommonBundle extends Bundle {
  val da = new Bundle() {
    val splitSpecFailIid = ValidIO(UInt(InstructionIdWidth.W))
  }
  val wb = new Bundle() {
    val iid : UInt = UInt(InstructionIdWidth.W)
    val cmplt : Bool = Bool()
    val abnormal : Bool = Bool()
    val breakPointAData : Bool = Bool()
    val breakPointBData : Bool = Bool()
    val exceptVec = ValidIO(UInt(ExceptionVecWidth.W))
    val flush : Bool = Bool()
    val mtval : UInt = UInt(MtvalWidth.W)
    val noSpecHit : Bool = Bool()
    val noSpecMispred : Bool = Bool()
    val noSpecMiss : Bool = Bool()
    val specFail : Bool = Bool()
    val vstart = ValidIO(UInt(VlmaxBits.W))
    val vsetvl : Bool = Bool()
  }
}

abstract class RobToRetireInstBundle extends Bundle {
  val valid : Bool = Bool()
  val bju : Bool = Bool()
  val conditionalBranch : Bool = Bool()
  val conditionalBranchTaken : Bool = Bool()
  val jmp : Bool = Bool()
  val load : Bool = Bool()
  val split : Bool = Bool()
  val store : Bool = Bool()
  val vlPred : Bool = Bool()
  val vsetvli : Bool = Bool()
  val fpDirty : Bool = Bool()
  val vecDirty : Bool = Bool()
  val noSpecHit : Bool = Bool()
  val noSpecMispred : Bool = Bool()
  val noSpecMiss : Bool = Bool()
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

abstract class RobToRetireInstExtraBundle extends RobToRetireInstBundle {
  val bhtMispred : Bool = Bool()
  val bjuIncPc : UInt = UInt(PcWidth.W)
  val breakPoint : Bool = Bool()
  val ctcFlush : Bool = Bool()
  val dataBreakPoint : Bool = Bool()
  val debugDisable : Bool = Bool()
  val efPcValid : Bool = Bool()
  val exceptionVec = ValidIO(UInt(ExceptionVecWidth.W))
  val highHwException : Bool = Bool()
  val iid : UInt = UInt(InstructionIdWidth.W)
  val instMmuException : Bool = Bool()
  val instBreakPoint : Bool = Bool()
  val instFlush : Bool = Bool()
  val interruptVec = ValidIO(UInt(interruptVecWidth.W))
  val interruptMask : Bool = Bool()
  val jmpMispred : Bool = Bool()
  val mtval : UInt = UInt(MtvalWidth.W)

  val pcal : Bool = Bool()
  val pret : Bool = Bool()
  val ras : Bool = Bool()
  val specFail : Bool = Bool()
  val specFailNoSsf : Bool = Bool()
  val specFailSsf : Bool = Bool()
  val vsetvl : Bool = Bool()
  val vstart = ValidIO(UInt(VlmaxBits.W))
}

abstract class RobToRetireBundle extends Bundle {
  // Todo: imm
  val commit : Vec[Bool] = Vec(NumCommitEntry, Bool())
  val ctcFlushSrtEn : Bool = Bool()
  val inst0 : RobToRetireInstExtraBundle = new RobToRetireInstExtraBundle {}
  val inst1 : RobToRetireInstBundle = new RobToRetireInstBundle {}
  val inst2 : RobToRetireInstBundle = new RobToRetireInstBundle {}
  val robCurPc : UInt = UInt(PcWidth.W)
}

class RobToCpu extends Bundle {
  val noRetire : Bool = Bool()
}

class RobToHad extends Bundle {
  val breakpointDataSt : Bool = Bool()
  val dataBreakpointAValid : Bool = Bool()
  val dataBreakpointBValid : Bool = Bool()
  val instBreakpointInstValid : Bool = Bool()
  val instBreakpointAValid : Bool = Bool()
  val instBreakpointBValid : Bool = Bool()
  val instExeDead : Bool = Bool()
  val inseSplit : Bool = Bool()
  // Todo: imm
  val instNonIrvBreakpoint : Vec[UInt] = Vec(3, UInt(4.W))
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
  val retirePc : Vec[ValidIO[UInt]] = Vec(NumRetireEntry, ValidIO(UInt(PcWidth.W)))
}

class RobYyXx extends Bundle {
  // Todo: check
  val commitIid : Vec[ValidIO[UInt]] = Vec(NumCommitEntry, ValidIO(UInt(InstructionIdWidth.W)))
  val retire : Vec[Bool] = Vec(NumRetireEntry, Bool())
}

class RegEntryFromIfuBundle extends Bundle {
  val xxSyncReset : Bool = Bool()
}