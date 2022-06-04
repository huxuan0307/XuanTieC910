package Core.RTU

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.AddrConfig._
import Core.Config.PcStart
import Core.IntConfig._
import Core.ExceptionConfig._
import Core.PipelineConfig._
import Core.VectorUnitConfig._
import chisel3.util.experimental.BoringUtils
import difftest._

class RtuFromCp0Bundle extends Bundle {
  val srtEn : Bool = Bool()
  val icgEn : Bool = Bool()
  val xxIntB : Bool = Bool()
  val xxVec : UInt = UInt(InterruptVecWidth.W)
  val yyClkEn : Bool = Bool()
}

class RtuFromHadBundle extends Bundle {
  val debugDisable : Bool = Bool()
  val debugReqEn : Bool = Bool()
  val debugRetireInfoEn : Bool = Bool()
  val eventDebugReq : Bool = Bool()
  // Todo: figure out
  val fdb : Bool = Bool()
  val hwDebugReq : Bool = Bool()
  val hwDebugReqGateClk : Bool = Bool()
  val nonIrvBreakpointDebugReq : Bool = Bool()
  val pop1Disable : Bool = Bool()
  val traceDebugReq : Bool = Bool()
  val traceEn : Bool = Bool()
  val xxJdbReq : Bool = Bool()
  val yyXxExitDebug : Bool = Bool()
  val dataBreakPointDebugReq : Bool = Bool()
  val instBreakPointDebugReq : Bool = Bool()
  val xxTme : Bool = Bool()
}

class RtuFromIduBundle extends Bundle {
  class RtuFromIrBundle extends Bundle {
    class RtuFromIrAllocBundle extends Bundle {
      val allocValidVec : Vec[Bool] = Vec(NumCreateEntry, Bool())
      val allocGateClkValid : Bool = Bool()
    }
    val preg = new RtuFromIrAllocBundle
    val ereg = new RtuFromIrAllocBundle
    val freg = new RtuFromIrAllocBundle
    val vreg = new RtuFromIrAllocBundle
  }
  class RtuFromIduPstBundle extends Bundle {
    val preg : Vec[PstPregEntryCreateBundle] = Vec(NumCreateEntry, new PstPregEntryCreateBundle)
    val vfreg : Vec[PstVFregEntryCreateBundle] = Vec(NumCreateEntry, new PstVFregEntryCreateBundle)
    val ereg : Vec[PstEregEntryCreateBundle] = Vec(NumCreateEntry, new PstEregEntryCreateBundle)
    val pregDeallocMaskOH : Vec[Bool] = Vec(NumPhysicRegs, Bool())
    val vregDeallocMaskOH : Vec[Bool] = Vec(NumVPregs, Bool())
    val fregDeallocMaskOH : Vec[Bool] = Vec(NumFPregs, Bool())
  }

  val fenceIdle : Bool = Bool()
  val fromIr = new RtuFromIrBundle
  val toPst = new RtuFromIduPstBundle
  val robCreate : Vec[RobCreateBundle] = Vec(NumCreateEntry, new RobCreateBundle)
}

class RtuFromIfuBundle extends Bundle {
  val xxSyncReset : Bool = Bool()
  val curPc : UInt = UInt(PcWidth.W)
  val curPcLoad : Bool = Bool()
}

class ToRobPipeCtrlBundle extends Bundle {
  // Common
  val iid : UInt = UInt(InstructionIdWidth.W)
  val cmplt : Bool = Bool()
  // Abnormal
  val abnormal : Bool = Bool()
  // Special
  val breakPoint : Bool = Bool()
  val efPc = ValidIO(UInt(PcWidth.W))
  val exceptionVec = ValidIO(UInt(ExceptionVecWidth.W))
  val flush : Bool = Bool()
  val highHwException : Bool = Bool()
  val instMmuException : Bool = Bool()
  val mtval : UInt = UInt(MtvalWidth.W)
  val vsetvl : Bool = Bool()
  val vstart = ValidIO(UInt(VlmaxBits.W))
  // Branch
  val bhtMispred : Bool = Bool()
  val jmpMispred : Bool = Bool()
  // Lsu data alignment
  val splitSpecFailIid = ValidIO(UInt(InstructionIdWidth.W))
  // Lsu write back
  val breakpointData = new RobBreakpointDataBundle
  val exceptVec = ValidIO(UInt(ExceptionVecWidth.W))
  val noSpec = new RobNoSpecBundle
  val specFail : Bool = Bool()
}

class RtuFromIuBundle extends Bundle {
  /**
   * iu_rtu_ex2_pipe0_wb_preg_vld
   * iu_rtu_ex2_pipe0_wb_preg_expand
   * iu_rtu_ex2_pipe1_wb_preg_vld
   * iu_rtu_ex2_pipe1_wb_preg_expand
   */
  val wbData : Vec[ValidIO[Vec[Bool]]] = Vec(NumIuPipe, ValidIO(Vec(NumPhysicRegs, Bool())))
  val pcFifoPopDataVec : Vec[PcFifoData] = Vec(NumPopEntry, new PcFifoData)
  // Todo: imm
  val pipeCtrlVec : Vec[ToRobPipeCtrlBundle] = Vec(3, new ToRobPipeCtrlBundle)
}

class RtuFromLsuBundle extends Bundle {
  val allCommitDataValid : Bool = Bool()
  // Todo: imm
  val asyncExceptAddr : UInt = UInt(40.W)
  val asyncExceptValid : Bool = Bool()
  val ctcFlushValid : Bool = Bool()
  /**
   * load pipe wb: lsu_rtu_wb_pipe3_wb_preg_vld, lsu_rtu_wb_pipe3_wb_preg_expand
   */

  val wbPregData : Vec[ValidIO[Vec[Bool]]] = Vec(NumLuPipe, ValidIO(Vec(NumPhysicRegs, Bool())))
  val wbVFregData = new Bundle {
    val fregValid : Bool = Bool()
    val vregValid : Bool = Bool()
    val pregOH : Vec[Bool] = Vec(NumVPregs, Bool())
  }
  // Todo: imm
  val pipeCtrlVec : Vec[ToRobPipeCtrlBundle] = Vec(2, new ToRobPipeCtrlBundle)
}

class RtuFromVfpuBundle extends Bundle {
  // Todo:

}

class RtuTopInput extends Bundle {
  val fromCp0 = new RtuFromCp0Bundle
  val fromHad = new RtuFromHadBundle
  val fromHpcp = new RtuFromHpcpBundle
  val fromIdu = new RtuFromIduBundle
  val fromIfu = new RtuFromIfuBundle
  val fromIu = new RtuFromIuBundle
  val fromLsu = Output(new RtuFromLsuBundle)
  val fromMmu = new RetireFromMmuBundle
  val fromPad = new RtuFromPadBundle
  val fromVfpu = new RtuFromVfpuBundle
}

class RtuToHadBundle extends Bundle {
  // from rob
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
  // from retire
  val debugAckInfo : Bool = Bool()
  val debugReqAck : Bool = Bool()
  val inst0BreakpointInst : Bool = Bool()
  val debugAckPc : Bool = Bool()
  val mBreakpointDataAck : Bool = Bool()
  val mBreakpointInstAck : Bool = Bool()
  val pc : UInt = UInt(PcWidth.W)
  class RtuToHadPcFifo extends Bundle {
    val changeFlow : Bool = Bool()
    val condBr : Bool = Bool()
    val condBrTaken : Bool = Bool()
    val jmp : Bool = Bool()
    val npc : UInt = UInt(PcWidth.W)
    val pCall : Bool = Bool()
    val pReturn : Bool = Bool()
  }
  val pcFifoData = new Bundle {
    val instVec : Vec[RtuToHadPcFifo] = Vec(3, new RtuToHadPcFifo)
    val inst0Iid : UInt = UInt(InstructionIdWidth.W)
  }
  val splitInst : Bool = Bool()
  class DebugInfo extends Bundle {
    // Todo: imm
    val retiredRegWb = Vec(3, Bool())
    val robFull : Bool = Bool()
    val read0Iid : UInt = UInt(InstructionIdWidth.W)
    val create0Iid : UInt = UInt(InstructionIdWidth.W)
    // Todo: imm
    val entryNum : UInt = UInt(7.W)
    // Todo: imm
    val robPcCur : UInt = UInt(7.W)
    val commit0 : Bool = Bool()
    val commitStNoValid : Bool = Bool()
    val flushState : UInt = UInt(FlushState.width.W)
    val aeState : UInt = UInt(AsyncExceptState.width.W)
    val ssfState : UInt = UInt(SsfState.width.W)
  }
  // from pst
  val instNotWb : Bool = Bool()
  // common
  val debugInfo = new DebugInfo
}

class RtuToHpcpBundle extends Bundle {
  val rob = new RobToHpcp
  val trace = new RetireToHpcp
}

class RtuToIduBundle extends Bundle {
  // pst
  val pregAlloc : Vec[ValidIO[UInt]] = Vec(NumCreateEntry, ValidIO(UInt(NumPhysicRegsBits.W)))
  val eregAlloc : Vec[ValidIO[UInt]] = Vec(NumCreateEntry, ValidIO(UInt(NumEregsBits.W)))
  val fregAlloc : Vec[ValidIO[UInt]] = Vec(NumCreateEntry, ValidIO(UInt(NumFPregsBits.W)))
  val vregAlloc : Vec[ValidIO[UInt]] = Vec(NumCreateEntry, ValidIO(UInt(NumVPregsBits.W)))
  val pstEmpty : Bool = Bool()
  val rtRecoverPreg : Vec[UInt] = Vec(NumLogicRegs, UInt(NumPhysicRegsBits.W))
  // Todo:
  // retire
  val flushFe : Bool = Bool()
  val flushIs : Bool = Bool()
  val flushStall : Bool = Bool()
  val retire0InstValid : Bool = Bool()
  val srtEn : Bool = Bool()
  // rob
  val interruptValid : Bool = Bool()
  val robEmpty : Bool = Bool()
  val robFull : Bool = Bool()
  // Todo: check imm
  val robInstIidVec : Vec[UInt] = Vec(NumCreateEntry, UInt(InstructionIdWidth.W))
}

class RtuToIuBundle extends Bundle {
  // Todo: check imm
  val robReadPcFifoValid : Vec[Bool] = Vec(NumRobReadEntry, Bool())
  val robReadPcFifoGateClkValid : Bool = Bool()
  val flushChangeFlowMask : Bool = Bool()
  val flushFe : Bool = Bool()
}

class RtuToLsuBundle extends Bundle {
  // Todo: check imm
  val commitIidUpdateVal : Vec[UInt] = Vec(NumRetireEntry, UInt(InstructionIdWidth.W))
  val asyncFlush : Bool = Bool()
  val eretFlush : Bool = Bool()
  val exceptFlush : Bool = Bool()
  val specFailFlush : Bool = Bool()
  val specFailIid : UInt = UInt(InstructionIdWidth.W)
}

class RtuYyXx extends Bundle {
  // rob
  // Todo: check imm
  val commitIid : Vec[ValidIO[UInt]] = Vec(NumCommitEntry, ValidIO(UInt(InstructionIdWidth.W)))
  val retire : Vec[Bool] = Vec(NumRetireEntry, Bool())
  // retire
  val debugOn : Bool = Bool()
  val exceptVec : UInt = UInt(6.W)
  val flush : Bool = Bool()
  val retire0Normal : Bool = Bool()
}

class RtuTopOutput extends Bundle {
  val toCp0 = new RtuToCp0Bundle
  val toHad = new RtuToHadBundle
  val toHpcp = new RtuToHpcpBundle
  val toIdu = new RtuToIduBundle
  val toIfu = new RetireToIfuBundle
  val toIu = new RtuToIuBundle
  val toLsu = new RtuToLsuBundle
  val toMmu = new RetireToMmuBundle
  val toPad = new RobToPad
  val yyXx = new RtuYyXx
}

class RtuTopIO extends Bundle {
  val in : RtuTopInput = Flipped(Output(new RtuTopInput))
  val out : RtuTopOutput = Output(new RtuTopOutput)
}

class RtuTop extends Module {
  val io : RtuTopIO = IO(new RtuTopIO)

  private val pstPreg = Module(new PstPreg)
  private val rob = Module(new Rob)
  private val retire = Module(new Retire)

  pstPreg match {
    case pstPreg: PstPreg =>
      val in = pstPreg.io.in
      in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
      in.fromCp0.rtuIcgEn := io.in.fromCp0.icgEn
      in.fromIdu.alloc.zipWithIndex.foreach{
        case (value, i) =>
          value.valid := io.in.fromIdu.fromIr.preg.allocValidVec(i)
          value.bits := io.in.fromIdu.toPst.preg(i)
      }
      in.fromIdu.allocGateClkValid := io.in.fromIdu.fromIr.preg.allocGateClkValid
      in.fromIdu.deallocMaskOH := io.in.fromIdu.toPst.pregDeallocMaskOH
      in.fromIfu.xxSyncReset := io.in.fromIfu.xxSyncReset
      in.fromIuEx2.wbData := io.in.fromIu.wbData
      in.fromLsu.wbData := io.in.fromLsu.wbPregData
      in.fromPad.yyIcgScanEn := io.in.fromPad.yyIcgScanEn
      in.retiredEregWb := true.B // Todo: move this port out of pstPreg
      in.retiredFregWb := true.B // Todo: move this port out of pstPreg
      in.retiredVregWb := true.B // Todo: move this port out of pstPreg
      in.fromRetire.asyncFlush := retire.io.out.toPst.asyncFlush
      in.fromRetire.wbRetireInstPregValid := retire.io.out.toPst.wbInstPregValid
      in.fromRob.iidUpdate := rob.io.out.toPst.iidUpdate
      in.fromRob.gateClkValid := rob.io.out.toPst.gateClkValid
      in.fromRtu.yyXxFlush := retire.io.out.yyXx.flush
  }

  rob match {
    case rob: Rob =>
      val in = rob.io.in
      in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
      in.fromCp0.icgEn := io.in.fromCp0.icgEn
      in.fromCp0.xxVec := io.in.fromCp0.xxVec
      in.fromCp0.xxIntB := io.in.fromCp0.xxIntB
      in.fromHad.debugRetireInfoEn := io.in.fromHad.debugRetireInfoEn
      in.fromHad.debugReqEn := io.in.fromHad.debugReqEn
      in.fromHad.xxTme := io.in.fromHad.xxTme
      in.fromHad.dataBreakPointDebugReq := io.in.fromHad.dataBreakPointDebugReq
      in.fromHad.instBreakPointDebugReq := io.in.fromHad.instBreakPointDebugReq
      in.fromHpcp := io.in.fromHpcp
      in.fromIdu.fenceIdle := io.in.fromIdu.fenceIdle
      in.fromIdu.robCreate := io.in.fromIdu.robCreate
      in.fromIfu.curPc := io.in.fromIfu.curPc
      in.fromIfu.curPcLoad := io.in.fromIfu.curPcLoad
      in.fromIu.pcFifoPopDataVec := io.in.fromIu.pcFifoPopDataVec
      in.fromIu.pipe0 := io.in.fromIu.pipeCtrlVec(0)
      in.fromIu.pipe1 := io.in.fromIu.pipeCtrlVec(1)
      in.fromIu.pipe2 := io.in.fromIu.pipeCtrlVec(2)
      in.fromLsu.allCommitDataValid := io.in.fromLsu.allCommitDataValid
      in.fromLsu.pipe3 := io.in.fromLsu.pipeCtrlVec(0)
      in.fromLsu.pipe4 := io.in.fromLsu.pipeCtrlVec(1)
      in.fromPad.yyIcgScanEn := io.in.fromPad.yyIcgScanEn
      in.fromRetire := retire.io.out.toRob
      in.fromRtu.yyXxFlush := retire.io.out.yyXx.flush
      in.fromVfpu.pipe6 := DontCare // Todo
      in.fromVfpu.pipe7 := DontCare // Todo
  }

  retire match {
    case retire: Retire =>
      val in = retire.io.in
      in.fromCp0.yyClkEn                  := io.in.fromCp0.yyClkEn
      in.fromCp0.icgEn                    := io.in.fromCp0.icgEn
      in.fromCp0.srtEn                    := io.in.fromCp0.srtEn
      in.fromHad.debugDisable             := io.in.fromHad.debugDisable
      in.fromHad.debugReqEn               := io.in.fromHad.debugReqEn
      in.fromHad.eventDebugReq            := io.in.fromHad.eventDebugReq
      in.fromHad.fdb                      := io.in.fromHad.fdb
      in.fromHad.hwDebugReq               := io.in.fromHad.hwDebugReq
      in.fromHad.hwDebugReqGateClk        := io.in.fromHad.hwDebugReqGateClk
      in.fromHad.nonIrvBreakpointDebugReq := io.in.fromHad.nonIrvBreakpointDebugReq
      in.fromHad.pop1Disable              := io.in.fromHad.pop1Disable
      in.fromHad.traceDebugReq            := io.in.fromHad.traceDebugReq
      in.fromHad.traceEn                  := io.in.fromHad.traceEn
      in.fromHad.xxJdbReq                 := io.in.fromHad.xxJdbReq
      in.fromHad.yyXxExitDebug            := io.in.fromHad.yyXxExitDebug

      in.fromHpcp.cntEn                   := io.in.fromHpcp.cntEn

      in.fromLsu.allCommitDataValid := io.in.fromLsu.allCommitDataValid
      in.fromLsu.asyncExceptValid   := io.in.fromLsu.asyncExceptValid
      in.fromLsu.asyncExceptAddr    := io.in.fromLsu.asyncExceptAddr
      in.fromLsu.ctcFlushValid      := io.in.fromLsu.ctcFlushValid

      in.fromMmu.mmuEn := io.in.fromMmu.mmuEn

      in.fromPad.yyIcgScanEn := io.in.fromPad.yyIcgScanEn

      in.fromPst := pstPreg.io.out.toRetire

      in.fromRob := rob.io.out.toRetire
  }

  io.out.toCp0 := retire.io.out.toCp0

  io.out.toHad.breakpointDataSt         := rob.io.out.toHad.breakpointDataSt
  io.out.toHad.dataBreakpoint           := rob.io.out.toHad.dataBreakpoint
  io.out.toHad.instBreakpoint           := rob.io.out.toHad.instBreakpoint
  io.out.toHad.instBreakpointInstValid  := rob.io.out.toHad.instBreakpointInstValid
  io.out.toHad.instExeDead              := rob.io.out.toHad.instExeDead
  io.out.toHad.instSplit                := rob.io.out.toHad.instSplit
  io.out.toHad.instNonIrvBreakpoint     := rob.io.out.toHad.instNonIrvBreakpoint
  io.out.toHad.retireInstInfo           := rob.io.out.toHad.retireInstInfo
  io.out.toHad.robEmpty                 := rob.io.out.toHad.robEmpty
  io.out.toHad.xxMBreakpointChangeFlow  := rob.io.out.toHad.xxMBreakpointChangeFlow

  io.out.toHad.debugAckInfo         := retire.io.out.toHad.debugAckInfo
  io.out.toHad.debugReqAck          := retire.io.out.toHad.debugReqAck
  io.out.toHad.inst0BreakpointInst  := retire.io.out.toHad.inst0BreakpointInst
  io.out.toHad.debugAckPc           := retire.io.out.toHad.debugAckPc
  io.out.toHad.mBreakpointDataAck   := retire.io.out.toHad.mBreakpointDataAck
  io.out.toHad.mBreakpointInstAck   := retire.io.out.toHad.mBreakpointInstAck
  io.out.toHad.pc                   := retire.io.out.toHad.pc
  io.out.toHad.pcFifoData           := retire.io.out.toHad.pcFifoData
  io.out.toHad.splitInst            := retire.io.out.toHad.splitInst

  io.out.toHad.instNotWb            := pstPreg.io.out.toHad.instNotWb

  io.out.toHad.debugInfo.retiredRegWb := pstPreg.io.out.toTop.retiredRegWb
  io.out.toHad.debugInfo.robFull      := rob.io.out.toTop.robFull
  io.out.toHad.debugInfo.read0Iid     := rob.io.out.toTop.read0Iid
  io.out.toHad.debugInfo.create0Iid   := rob.io.out.toTop.create0Iid
  io.out.toHad.debugInfo.entryNum     := rob.io.out.toTop.entryNum
  io.out.toHad.debugInfo.robPcCur     := rob.io.out.toTop.robCurPc
  io.out.toHad.debugInfo.commit0      := rob.io.out.toTop.commit0
  io.out.toHad.debugInfo.commitStNoValid := rob.io.out.toTop.commitStNoValid
  io.out.toHad.debugInfo.flushState := rob.io.out.toTop.flushState
  io.out.toHad.debugInfo.aeState    := retire.io.out.toTop.aeState
  io.out.toHad.debugInfo.ssfState   := rob.io.out.toTop.ssfState

  io.out.toHpcp.rob       := rob.io.out.toHpcp
  io.out.toHpcp.trace     := retire.io.out.toHpcp
  io.out.toIdu.pregAlloc  := pstPreg.io.out.toIdu.alloc
  io.out.toIdu.eregAlloc  := DontCare // Todo
  io.out.toIdu.fregAlloc  := DontCare // Todo
  io.out.toIdu.vregAlloc  := DontCare // Todo
  io.out.toIdu.pstEmpty   := pstPreg.io.out.toIdu.empty
  io.out.toIdu.rtRecoverPreg := pstPreg.io.out.toIdu.rtRecoverPreg
  // Todo: more
  io.out.toIdu.flushFe    := retire.io.out.toIdu.flushFe
  io.out.toIdu.flushIs    := retire.io.out.toIdu.flushIs
  io.out.toIdu.flushStall := retire.io.out.toIdu.flushStall
  io.out.toIdu.retire0InstValid := retire.io.out.toIdu.retire0InstValid
  io.out.toIdu.srtEn      := retire.io.out.toIdu.srtEn

  io.out.toIdu.interruptValid := rob.io.out.toIdu.retireInterruptValid
  io.out.toIdu.robEmpty   := rob.io.out.toIdu.robEmpty
  io.out.toIdu.robFull    := rob.io.out.toIdu.robFull
  io.out.toIdu.robInstIidVec := rob.io.out.toIdu.robInstIidVec

  io.out.toIfu := retire.io.out.toIfu

  io.out.toIu.flushFe := retire.io.out.toIu.flushFe
  io.out.toIu.flushChangeFlowMask := retire.io.out.toIu.flushChangeFlowMask
  io.out.toIu.robReadPcFifoValid := rob.io.out.toIu.robReadPcFifoValid
  io.out.toIu.robReadPcFifoGateClkValid := rob.io.out.toIu.robReadPcFifoGateClkValid

  io.out.toLsu.commitIidUpdateVal := rob.io.out.toLsu.commitIidUpdateVal
  io.out.toLsu.asyncFlush   := retire.io.out.toLsu.asyncFlush
  io.out.toLsu.eretFlush    := retire.io.out.toLsu.eretFlush
  io.out.toLsu.exceptFlush  := retire.io.out.toLsu.exceptFlush
  io.out.toLsu.specFailFlush := retire.io.out.toLsu.specFailFlush
  io.out.toLsu.specFailIid  := retire.io.out.toLsu.specFailIid

  io.out.toMmu.badVpn       := retire.io.out.toMmu.badVpn
  io.out.toMmu.exceptValid  := retire.io.out.toMmu.exceptValid

  io.out.toPad.retirePc     := rob.io.out.toPad.retirePc

  io.out.yyXx.commitIid     := rob.io.out.yyXx.commitIid
  io.out.yyXx.retire        := rob.io.out.yyXx.retire

  io.out.yyXx.debugOn       := retire.io.out.yyXx.debugOn
  io.out.yyXx.exceptVec     := retire.io.out.yyXx.exceptVec
  io.out.yyXx.flush         := retire.io.out.yyXx.flush
  io.out.yyXx.retire0Normal := retire.io.out.yyXx.retire0Normal
}
