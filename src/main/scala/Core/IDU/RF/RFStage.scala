package Core.IDU.RF

import Core.ExceptionConfig.ExceptionVecWidth
import Core.IDU.DecodeTable.{AluDecodeTable, DefaultInst}
import Core.IDU.FuncUnit
import Core.IDU.IS.AiqConfig.NumSrcArith
import Core.IDU.IS.BiqConfig.NumSrcBr
import Core.IDU.IS.LsiqConfig.NumSrcLs
import Core.IDU.IS.SdiqConfig.NumSrcSd
import Core.IDU.IS.VfiqConfig.NumSrcVf
import Core.IDU.IS._
import Core.IDU.Opcode.Opcode
import Core.IDU.RF.PrfConfig.NumPregReadPort
import Core.IntConfig._
import Core.VectorUnitConfig._
import Utils.Bits.{sext, zext}
import chisel3._
import chisel3.util._

trait RFStageConfig {
  // aiq + biq + lsiq + sdiq + viq
  def NumIssueAiq = 2
  def NumIssueBiq = 1
  def NumIssueLsiq = 2
  def NumIssueSdiq = 1
  def NumIssueVfiq = 2
  def NumIssue : Int = NumIssueAiq + NumIssueBiq + NumIssueLsiq +
    NumIssueSdiq + NumIssueVfiq
  def NumAiqEntry = 8
  def NumBiqEntry = 12
  def NumLsiqEntry = 12
  def NumSdiqEntry = 12
  def NumVfiqEntry = 8
  // Todo
  def StorePipeNumSet = Seq(5)
  def LsuPipeNumSet = Seq(3, 4, 5)
  def BjuPipeNumSet = Seq(2)
}

class IssueEnBundle extends Bundle {
  val gateClkIssueEn  : Bool = Bool()
  val issueEn         : Bool = Bool()
}

class RFStageFromCp0Bundle extends Bundle {
  val iduIcgEn  : Bool = Bool()
  val yyClkEn   : Bool = Bool()
  val lsuFenceIBroadDis   : Bool = Bool()
  val lsuFenceRwBroadDis  : Bool = Bool()
  val lsuTlbBroadDis      : Bool = Bool()
}

class RFStageFromHpcpBundle extends Bundle {
  val iduCntEn  : Bool = Bool()
}

class RFStageFromIuBundle extends Bundle {
  val stall = new Bundle {
    val divWb   : Bool = Bool()
    val mulEx1  : Bool = Bool()
  }
}

class RFStageFromVfpuBundle extends Bundle {
  val stall = new Bundle {
    val vdivWb : Bool = Bool()
  }
}

class RFStageFromRtuBundle extends Bundle {
  val flush = new Bundle {
    val fe : Bool = Bool()
    val is : Bool = Bool()
  }
  // Todo: check if need be merged into flush bundle
  val yyXxFlush : Bool = Bool()
}

class RFStageCtrlInput extends Bundle with RFStageConfig {
  val fromCp0   = new RFStageFromCp0Bundle
  val fromHpcp  = new RFStageFromHpcpBundle
  val fromIu    = new RFStageFromIuBundle
  val issueEnVec : Vec[IssueEnBundle] = Vec(NumIssue, new IssueEnBundle)
  val fromPad   = new PrfFromPadBundle
  val fromRtu   = new RFStageFromRtuBundle
  val fromVfpu  = new RFStageFromVfpuBundle
}

class RFStageToIqBundle extends Bundle with RFStageConfig {
  val launchFailValid : Bool = Bool()
  val popValid        : Bool = Bool()
  // Todo: add forward signal
  //  e.g. ctrl_aiq0_rf_pipe0_alu_reg_fwd_vld ctrl_viq1_rf_pipe7_vmla_vreg_fwd_vld
  val popDlbValid     : Bool = Bool()
  val stall           : Bool = Bool()
  /**
   *   Only for sdiq
   */
  val stAddrReadySet  : Bool = Bool()
  // Todo: Add launch valid dup
  //  e.g. ctrl_xx_rf_pipe0_preg_lch_vld_dup0
}

class RFStageToCp0Bundle extends Bundle {
  // ctrl
  // Todo: figure out
  val gateClkSel : Bool = Bool()
  val sel     : Bool = Bool()
  // data
  val opcode  : UInt = UInt(Opcode.width.W)
  val iid     : UInt = UInt(InstructionIdWidth.W)
  // replace opcode with inst
  val inst    : UInt = UInt(InstBits.W)
  val preg    : UInt = UInt(NumPhysicRegsBits.W)
  val src0    : UInt = UInt(XLEN.W)
}

class RFStageToHpcpBundle extends Bundle with RFStageConfig {
  val instValid : Bool = Bool()
  class PipeSignal extends Bundle {
    val instValid           : Bool = Bool()
    val launchFailValid     : Bool = Bool()
    val regLaunchFailValid  : Bool = Bool()
  }
  val pipeVec : Vec[PipeSignal] = Vec(NumIssue, new PipeSignal)
}

class RFStageToExuGateClkBundle extends Bundle {
  // gateClkIssue
  // issue
  // gateClkSel
  // sel
  // cbusGateClkSel
}

class RFStageToFuCtrlBundle extends Bundle {
  val sel : Bool = Bool()
  val gateClkSel : Bool = Bool()
}

class RFStageCtrlOutput extends Bundle with RFStageConfig {
  val toIq    : Vec[RFStageToIqBundle] = Vec(NumIssue, new RFStageToIqBundle)
  val toCp0   = new RFStageToCp0Bundle
  val toHpcp  = new RFStageToHpcpBundle
  val toExu   = new RFStageToExuGateClkBundle
  val toBju   = new RFStageToFuCtrlBundle
  val toDiv   = new RFStageToFuCtrlBundle
  val toMul   = new RFStageToFuCtrlBundle
  val toSpecial = new RFStageToFuCtrlBundle
  val toAlu0  = new RFStageToFuCtrlBundle // pipe0
  val toAlu1  = new RFStageToFuCtrlBundle // pipe1
  val toLu    = new RFStageToFuCtrlBundle // pipe3
  val toSt    = new RFStageToFuCtrlBundle // pipe4
  val toSd    = new RFStageToFuCtrlBundle // pipe5
  val toVfpu0 = new RFStageToFuCtrlBundle // pipe6
  val toVfpu1 = new RFStageToFuCtrlBundle // pipe6
}

class RFStageCtrlBundle extends Bundle {
  val in  : RFStageCtrlInput  = Flipped(Output(new RFStageCtrlInput))
  val out : RFStageCtrlOutput = Output(new RFStageCtrlOutput)
}

class RFStageFromHadBundle extends Bundle {
  val iduWbBrData   : UInt = UInt(XLEN.W)
  val iduWbBrValid  : Bool = Bool()
}

class RFStageFromPadBundle extends Bundle {
  val yyIcgScanEn : Bool = Bool()
}

class RFStageDataInput extends Bundle with RFStageConfig {
  class AiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumAiqEntry.W)
    val issueReadData   : AiqEntryData = Output(new AiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class BiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumBiqEntry.W)
    val issueReadData   : BiqEntryData = Output(new BiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class LsiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumLsiqEntry.W)
    val issueReadData   : LsiqEntryData = Output(new LsiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class SdiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumSdiqEntry.W)
    val issueReadData   : SdiqEntryData = Output(new SdiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }
  class VfiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH    : UInt = UInt(NumVfiqEntry.W)
    val issueReadData   : VfiqEntryData = Output(new VfiqEntryData)
    val issueEn         : Bool = Bool()
    val issueGateClkEn  : Bool = Bool()
  }

  val aiq0 = new AiqIssueBundle
  val aiq1 = new AiqIssueBundle
  val biq = new BiqIssueBundle
  val lsiq0 = new LsiqIssueBundle
  val lsiq1 = new LsiqIssueBundle
  val sdiq = new SdiqIssueBundle
  val vfiq0 = new VfiqIssueBundle
  val vfiq1 = new VfiqIssueBundle
  val fromHad = new RFStageFromHadBundle
  val fromPad = new RFStageFromPadBundle
}

class RFStageToFuDataBundle extends Bundle with RFStageConfig {
  val aluShort : Bool = Bool()
  val dstPreg = ValidIO(UInt(NumPhysicRegsBits.W))
  val dstVPreg = ValidIO(UInt(NumVPregsBits.W))
  val exceptVec = ValidIO(UInt(ExceptionVecWidth.W))
  // has replaced func with opcode
  val opcode      : UInt = UInt(Opcode.width.W)
  val highHwExpt  : Bool = Bool()
  val iid         : UInt = UInt(InstructionIdWidth.W)
  // Todo: imm
  val imm         : UInt = UInt(6.W)
  // output  [31 :0]  idu_iu_rf_pipe0_opcode;
  val inst        : UInt = UInt(InstBits.W)
  // Todo: imm
  // fifo id, ifu -> IU 拿到pid -> ifu
  val pid         : UInt = UInt(5.W)
  val rsltSel     : UInt = UInt(21.W)
  val specialImm  : UInt = UInt(20.W)
  val src0        : UInt = UInt(XLEN.W)
  val src1        : UInt = UInt(XLEN.W)
  val src1NoImm   : UInt = UInt(XLEN.W)
  val src2        : UInt = UInt(XLEN.W)
  val vl          : UInt = UInt(VlmaxBits.W)
  val vlmul       : UInt = UInt(VlmulBits.W)
  val vsew        : UInt = UInt(VsewBits.W)
}

class RFStageDataOutput extends Bundle with RFStageConfig {
  class RFStageToIqBundle(numEntry: Int, numSrc: Int) extends Bundle {
    val launchEntryOH : UInt = UInt(numEntry.W)
    val readyClr      : Vec[Bool] = Vec(numSrc, Bool())
  }
  val toAiq0  = new RFStageToIqBundle(NumAiqEntry, NumSrcArith)
  val toAiq1  = new RFStageToIqBundle(NumAiqEntry, NumSrcArith)
  val toBiq   = new RFStageToIqBundle(NumBiqEntry, NumSrcBr)
  val toLsiq0 = new RFStageToIqBundle(NumLsiqEntry, NumSrcLs)
  val toLsiq1 = new RFStageToIqBundle(NumLsiqEntry, NumSrcLs)
  val toSdiq  = new RFStageToIqBundle(NumSdiqEntry, NumSrcSd)
  val toVfiq0 = new RFStageToIqBundle(NumVfiqEntry, NumSrcVf)
  val toVfiq1 = new RFStageToIqBundle(NumVfiqEntry, NumSrcVf)

  val toIu = new RFStageToFuDataBundle
}

class RFStageDataBundle extends Bundle {
  val r   : Vec[PrfReadBundle]  = Flipped(Vec(NumPregReadPort, new PrfReadBundle))
  val in  : RFStageDataInput    = Flipped(Output(new RFStageDataInput))
  val out : RFStageDataOutput   = Output(new RFStageDataOutput)
}

class RFStageIO extends Bundle {
  val data = new RFStageDataBundle
  val ctrl = new RFStageCtrlBundle
}

class RFStage extends Module with RFStageConfig {
  val io : RFStageIO = IO(new RFStageIO)

  private val rtu = io.ctrl.in.fromRtu
  private val aiq0_data = io.data.in.aiq0
  private val aiq1_data = io.data.in.aiq1
  private val biq_data = io.data.in.biq

  // update if issue enable

  private val aiq0ReadData  = RegEnable(io.data.in.aiq0.issueReadData, io.data.in.aiq0.issueEn)
  private val aiq1ReadData  = RegEnable(io.data.in.aiq1.issueReadData, io.data.in.aiq1.issueEn)
  private val biqReadData   = RegEnable(io.data.in.biq.issueReadData, io.data.in.biq.issueEn)
  private val lsiq0ReadData = RegEnable(io.data.in.lsiq0.issueReadData, io.data.in.lsiq0.issueEn)
  private val lsiq1ReadData = RegEnable(io.data.in.lsiq1.issueReadData, io.data.in.lsiq1.issueEn)
  /**
   * Regs
   */

  // performance monitor
  private val pipeInstValidVec = RegInit(VecInit(Seq.fill(NumIssue)(false.B)))
  private val pipeLaunchFailValidVec = RegInit(VecInit(Seq.fill(NumIssue)(false.B)))
  private val lsuLaunchFailValidVec = RegInit(VecInit(Seq.fill(NumIssueLsiq + NumIssueSdiq)(false.B)))

  //==========================================================
  //                 RF Inst Valid registers
  //==========================================================
  //----------------------------------------------------------
  //                Pipe0 Instruction Valid
  //----------------------------------------------------------
  private val aiq0IssueLaunchPreg = Wire(Bool())
  private val aiq0IssueSpecial = Wire(Bool())
  private val aiq0IssueDiv = Wire(Bool())
  aiq0IssueLaunchPreg := false.B //////todo: tem for firrtl
  aiq0IssueSpecial := false.B //////todo: tem for firrtl
  aiq0IssueDiv := false.B //////todo: tem for firrtl

  //alu and special should set ready for pipe0 ex2 fwd
  //pipe0 ex1 fwd is set by alu reg fwd vld

  private val aiq0IssueAluRegValid = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.launchPreg
  private val aiq0IssueSpecialValid = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.special
  // Todo: div gateclk
  private val iuIsDivIssue = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.div

  private val pipe0PregLaunchValid = RegInit(false.B)
  private val pipe0SpecialValid = RegInit(false.B)

  //----------------------------------------------------------
  //                Pipe1 Instruction Valid
  //----------------------------------------------------------

  private val pipe1PregLaunchValid = RegInit(false.B)

  private val aiq1IssueAluRegValid = io.ctrl.in.issueEnVec(1).issueEn &&
    io.data.in.aiq1.issueReadData.launchPreg

  //----------------------------------------------------------
  //                Pipe2 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe3 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe4 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe5 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe6 Instruction Valid
  //----------------------------------------------------------

  private val pipe6vmlaPregLaunchValid = RegInit(false.B)
  private val viq0IssueVmlaRfValid = io.ctrl.in.issueEnVec(6).issueEn &&
    io.data.in.vfiq0.issueReadData.dstVregValid &&
    io.data.in.vfiq0.issueReadData.vmla

  //----------------------------------------------------------
  //                Pipe7 Instruction Valid
  //----------------------------------------------------------

  private val pipe7vmlaPregLaunchValid = RegInit(false.B)
  private val viq1IssueVmlaRfValid = io.ctrl.in.issueEnVec(7).issueEn &&
    io.data.in.vfiq1.issueReadData.dstVregValid &&
    io.data.in.vfiq1.issueReadData.vmla
  pipeInstValidVec.zipWithIndex.foreach {
    case (instValid, i) =>
      // pipe 5 rf stage is flush by flush_be
      // st pipe
      if (StorePipeNumSet.contains(i)) {
        when(rtu.yyXxFlush) {
          instValid := false.B
        }.otherwise {
          instValid := io.ctrl.in.issueEnVec(i).issueEn
        }
      }
      else {
        when(rtu.flush.fe || rtu.flush.is) {
          instValid := false.B
        }.otherwise {
          instValid := io.ctrl.in.issueEnVec(i).issueEn
        }
      }
  }

  when(rtu.flush.fe || rtu.flush.is) {
    pipe0PregLaunchValid  := false.B
    pipe0SpecialValid     := false.B
    pipe1PregLaunchValid  := false.B
    pipe6vmlaPregLaunchValid := false.B
    pipe7vmlaPregLaunchValid := false.B
  }.otherwise {
    pipe0PregLaunchValid  := aiq0IssueAluRegValid
    pipe0SpecialValid     := aiq0IssueSpecialValid
    pipe1PregLaunchValid  := aiq1IssueAluRegValid
    pipe6vmlaPregLaunchValid := viq0IssueVmlaRfValid
    pipe7vmlaPregLaunchValid := viq1IssueVmlaRfValid
  }

  //==========================================================
  //                RF Launch Ready registers
  //==========================================================
  //----------------------------------------------------------
  //                Pipe0 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd
  //  val pipe0AluRegFwdValidVec = Vec()

  private val aiq0IssueAluFwdInst = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.dstValid &&
    io.data.in.aiq0.issueReadData.aluShort

  //----------------------------------------------------------
  //                Pipe1 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  private val aiq1IssueAluFwdInst = io.ctrl.in.issueEnVec(1).issueEn &&
    io.data.in.aiq1.issueReadData.dstValid &&
    io.data.in.aiq1.issueReadData.aluShort
  // Todo:
//  val aiq1IssueMlaFwdInst = io.ctrl.in.issueEnVec(1).issueEn &&
//    io.data.in.aiq1.issueReadData.mlaValid

  //----------------------------------------------------------
  //                Pipe6 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  private val viq0IssueVmlaFwdInst = io.ctrl.in.issueEnVec(6).issueEn &&
    io.data.in.vfiq0.issueReadData.vmlaShort

  //----------------------------------------------------------
  //                Pipe7 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  private val viq1IssueVmlaFwdInst = io.ctrl.in.issueEnVec(7).issueEn &&
    io.data.in.vfiq1.issueReadData.vmlaShort

  //==========================================================
  //                     RF Stall Signals
  //==========================================================
  //----------------------------------------------------------
  //                     VR/GPR move stall
  //----------------------------------------------------------
  //ex2 pipe0/1 mtvr will share ex1 pipe6/7
  //stall viq all inst issue at pipe0/1 rf

  private val pipe0MtvrValid = pipeInstValidVec(0) && io.data.in.aiq0.issueReadData.mtvr
  private val pipe1MtvrValid = pipeInstValidVec(1) && io.data.in.aiq1.issueReadData.mtvr

  private val pipe6MfvrValid = pipeInstValidVec(6) && io.data.in.vfiq0.issueReadData.mfvr
  private val pipe7MfvrValid = pipeInstValidVec(7) && io.data.in.vfiq1.issueReadData.mfvr

  //----------------------------------------------------------
  //                     div/vdiv stall
  //----------------------------------------------------------
  //when div need to write back, it will stall issue at wb pipe
  private val pipe0DivStall = io.ctrl.in.fromIu.stall.divWb
  private val pipe6VdivStall = io.ctrl.in.fromVfpu.stall.vdivWb

  //----------------------------------------------------------
  //                        mul stall
  //----------------------------------------------------------
  private val pipe1MulStall = io.ctrl.in.fromIu.stall.mulEx1

  //----------------------------------------------------------
  //                 IU special cmplt stall
  //----------------------------------------------------------
  //when IU special need cmplt at EX1, it will share pipe0
  //RF cmplt port. it will not be conflict with wb port share,
  //so do not need lch fail
  private val pipe0SpecialStall = pipe0SpecialValid

  //----------------------------------------------------------
  //              Output Stall Signals to IQ
  //----------------------------------------------------------
  // Note: aiq0 --> viq0, aiq1 --> viq1
  // aiq0
  io.ctrl.out.toIq(0).stall := pipe6MfvrValid || pipe0DivStall || pipe0SpecialStall
  io.ctrl.out.toIq(1).stall := pipe7MfvrValid
  io.ctrl.out.toIq(2).stall := DontCare
  io.ctrl.out.toIq(3).stall := DontCare
  io.ctrl.out.toIq(4).stall := DontCare
  io.ctrl.out.toIq(5).stall := DontCare
  // viq0
  io.ctrl.out.toIq(6).stall := pipe0MtvrValid || pipe6VdivStall
  io.ctrl.out.toIq(7).stall := pipe1MtvrValid

  //==========================================================
  //                   Launch fail Signals
  //==========================================================
  private val pipeLaunchFailVec = Wire(Vec(NumIssue, Bool()))
  // Todo: figure this CAUTION
  //CAUTION: avoid dead lock: when inst1 lch fail inst2, inst2
  //         may lch fail inst1 through src no ready

  //----------------------------------------------------------
  //                  div/vdiv wb lch fail
  //----------------------------------------------------------
  //pipe6 vdiv stall may be conflict with pipe0 mtvr stall
  //pipe0 div stall may be conflict with pipe6 mfvr stall
  //so launch fail mfvr/mtvr at this situation
  private val pipe0VdivMtvrLaunchFail = pipe0MtvrValid && pipe6VdivStall
  private val pipe6DivMfvrLaunchFail = pipe6MfvrValid && pipe0DivStall

  //----------------------------------------------------------
  //                mult mfvr share lch fail
  //----------------------------------------------------------
  //pipe1 ex2 mult will share pipe1 rf
  //pipe7 ex2 mfvr will share pipe1 ex1,
  //  (which is pipe7 ex1 share pipe1 rf)
  //pipe1 ex2 mult will be conflict with pipe7 ex1 mfvr
  //so launch fail pipe7 rf mfvr when pipe1 ex1 is mult
  private val pipe7mulMfvrLaunchFail = pipe7MfvrValid && pipe1MulStall

  //----------------------------------------------------------
  //               preg read port share lch fail
  //----------------------------------------------------------
  //prepare src vld signals
  private val pipe0src2Valid = pipeInstValidVec(0) && io.data.in.aiq0.issueReadData.srcValid(2)
  private val pipe1src2Valid = pipeInstValidVec(1) && io.data.in.aiq1.issueReadData.srcValid(2)
  private val pipe3src1Valid = pipeInstValidVec(3) && io.data.in.lsiq0.issueReadData.srcValid(1)
  private val pipe5src0Valid = pipeInstValidVec(5) && io.data.in.sdiq.issueReadData.src0Valid
  //preg read port share priority:
  //pipe0 src2 > pipe3 src1, pipe1 src1 > pipe5 src0
  //so launch fail pipe3/5 when pipe0/1 shares preg read port
  private val pipe3PregLaunchFail = pipe3src1Valid && pipe0src2Valid
  private val pipe5PregLaunchFail = pipe5src0Valid && pipe1src2Valid

  //----------------------------------------------------------
  //               vreg read port share lch fail
  //----------------------------------------------------------
  private val pipe3srcVmValid = pipeInstValidVec(3) && io.data.in.lsiq0.issueReadData.srcVmValid
  private val pipe4srcVmValid = pipeInstValidVec(4) && io.data.in.lsiq1.issueReadData.srcVmValid
  private val pipe6srcV2Valid = pipeInstValidVec(6) && io.data.in.vfiq0.issueReadData.srcValidVec(2)
  private val pipe7srcV2Valid = pipeInstValidVec(7) && io.data.in.vfiq1.issueReadData.srcValidVec(2)

  //vreg read port share priority:
  //pipe6 srcv2 > pipe3 srcvm, pipe7 srcv2 > pipe4 srcvm
  //so launch fail pipe3/4 when pipe6/7 vreg read port need to shared
  private val pipe3VregLaunchFail = pipe3srcVmValid && pipe6srcV2Valid
  private val pipe4VregLaunchFail = pipe4srcVmValid && pipe7srcV2Valid

  //----------------------------------------------------------
  //               unsplit vmlu share lch fail
  //----------------------------------------------------------
  // Todo: figure out
  //need to consider pipe7 lch fail, avoiding dead lock:
  //if pipe7 vmul unsplit lch fail pipe6 older inst x, older inst x
  //may lch fail pipe7 vmul unsplit through src no ready

  private val pipe7VmulUnsplitValid = pipeInstValidVec(7) &&
    !pipeLaunchFailVec(7) &&
    io.data.in.vfiq1.issueReadData.vmulUnsplit
  private val pipe6VmulValid = pipeInstValidVec(7) &&
    io.data.in.vfiq0.issueReadData.vmul
  //pipe7 vmul unsplit inst will share pipe6 vmul
  //so lch fail pipe6 vmul at this situation
  private val pipe6vmulUnsplitLaunchFail = pipe7VmulUnsplitValid && pipe6VmulValid

  //----------------------------------------------------------
  //                 Source Not Ready Signal
  //----------------------------------------------------------
  private val pipeSrcNotReadyVec = Wire(Vec(NumIssue, Bool()))
  pipeSrcNotReadyVec.foreach(_ := false.B)
  // Todo:
  //----------------------------------------------------------
  //                 Launch Fail Signals
  //----------------------------------------------------------
  //lch fail without src no rdy
  private val pipeOtherLaunchFailVec = WireInit(VecInit(Seq.fill(NumIssue)(false.B)))
  pipeOtherLaunchFailVec := DontCare // 1,2,
  pipeOtherLaunchFailVec(0) := pipe0VdivMtvrLaunchFail
  pipeOtherLaunchFailVec(3) := pipe3PregLaunchFail || pipe3VregLaunchFail
  pipeOtherLaunchFailVec(4) := pipe4VregLaunchFail
  pipeOtherLaunchFailVec(5) := pipe5PregLaunchFail
  pipeOtherLaunchFailVec(6) := pipe6DivMfvrLaunchFail || pipe6vmulUnsplitLaunchFail
  pipeOtherLaunchFailVec(7) := pipe7mulMfvrLaunchFail

  //should consider src no ready lch fail
  pipeLaunchFailVec.zipWithIndex.foreach {
    case (launchFail, i) =>
      launchFail := pipeSrcNotReadyVec(i) || pipeOtherLaunchFailVec(i)
  }

  //==========================================================
  //                    RF Control Signals
  //==========================================================
  //----------------------------------------------------------
  //                    RF pipedown valid
  //----------------------------------------------------------
  private val pipeDownValidVec = Wire(Vec(NumIssue, Bool()))

  pipeDownValidVec.zipWithIndex.foreach {
    case (pipeDownValid, i) =>
      pipeDownValid := pipeInstValidVec(i) && !pipeLaunchFailVec(i)
  }

  //----------------------------------------------------------
  //                 lch fail to clear iq frz
  //----------------------------------------------------------
  io.ctrl.out.toIq.zipWithIndex.foreach {
    case (iq, i) =>
      iq.launchFailValid := pipeInstValidVec(i) && pipeLaunchFailVec(i)
  }

  //----------------------------------------------------------
  //               no lch fail to pop iq entry
  //----------------------------------------------------------
  io.ctrl.out.toIq.zipWithIndex.foreach {
    case (iq, i) =>
      //pipe3/4/5 pop by LSU
      if (!LsuPipeNumSet.contains(i))
        iq.popValid := pipeDownValidVec(i)
      else
        iq.popValid := DontCare

      //pop singals for IR dlb, optimized for timing
      if (!LsuPipeNumSet.contains(i) || !BjuPipeNumSet.contains(i))
        iq.popDlbValid := pipeInstValidVec(i)
      else
        iq.popDlbValid := DontCare
  }

  //----------------------------------------------------------
  //                    lch fail for hpcp
  //----------------------------------------------------------
  //lch fail by src no rdy
  io.ctrl.out.toHpcp.pipeVec.zipWithIndex.foreach {
    case (pipe, i) =>
      pipe.launchFailValid := pipeInstValidVec(i) && pipeSrcNotReadyVec(i)
  }
  //lch fail by preg share
  io.ctrl.out.toHpcp.pipeVec.foreach(_.regLaunchFailValid := false.B)
  io.ctrl.out.toHpcp.pipeVec(3).regLaunchFailValid := pipeInstValidVec(3) &&
    (pipe3PregLaunchFail || pipe3VregLaunchFail)
  io.ctrl.out.toHpcp.pipeVec(4).regLaunchFailValid := pipeInstValidVec(4) &&
    pipe4VregLaunchFail
  io.ctrl.out.toHpcp.pipeVec(5).regLaunchFailValid := pipeInstValidVec(5) &&
    pipe5PregLaunchFail

  //----------------------------------------------------------
  //                staddr rdy set for SDIQ
  //----------------------------------------------------------
  //staddr replayed after DC stage should not set stdata ready, because
  //its stdata already poped after staddr signal valid at DC stage
  io.ctrl.out.toIq.foreach(_.stAddrReadySet := false.B)
  io.ctrl.out.toIq(5).stAddrReadySet := pipeInstValidVec(4) &&
    !pipeLaunchFailVec(4) && io.data.in.lsiq1.issueReadData.stAddr

  //==========================================================
  //                    RF stage Decoder
  //==========================================================

  private val aluDecodeTable = AluDecodeTable.table

  private val aiq0_inst = io.data.in.aiq0.issueReadData.inst
  private val funcUnit :: opcode :: rd :: rs1Vld :: rs2Vld = ListLookup(aiq0_inst, DefaultInst.inst, aluDecodeTable)
  // Todo: Add imm sel in decode table
  private val iu_imm_sel = Wire(Vec(5, Bool()))
  iu_imm_sel(0) := aiq0_inst(6,0) === "b0110111".U || aiq0_inst(6,0) === "b0010111".U
  iu_imm_sel(1) := aiq0_inst(1,0) === "b11".U && !iu_imm_sel(0)
  iu_imm_sel(2) := DontCare
  iu_imm_sel(3) := DontCare
  iu_imm_sel(4) := DontCare

  // Todo: imm_sel for RV64C

  private val iu_imm : UInt = MuxCase(0.U, Seq(
    iu_imm_sel(0) -> zext(XLEN, aiq0_inst(31, 12)),
    iu_imm_sel(1) -> sext(XLEN, aiq0_inst(31, 22)),
  ))

  //==========================================================
  //                    RF Read Regfile
  //==========================================================

  //----------------------------------------------------------
  //                  Read Regfile for IU
  //----------------------------------------------------------

  io.data.r(0).preg := aiq0ReadData.srcVec(0).preg
  io.data.r(1).preg := aiq0ReadData.srcVec(1).preg
  // Todo: fix r(2), should be pipe3 src1 data
  io.data.r(2).preg := aiq0ReadData.srcVec(2).preg
  io.data.r.zipWithIndex.foreach {
    case (rbundle, i) =>
      if (i > 2)
        rbundle.preg := DontCare
  }

  // iu src0 only from reg
  io.data.out.toIu.src0 := MuxCase(0.U, // Todo: replace with fwd data
    Seq(
      aiq0ReadData.srcVec(0).wb -> io.data.r(0).data,   // has write back -> read reg
    )
  )

  // iu src1 from reg or imm
  io.data.out.toIu.src1 := MuxCase(0.U, // Todo: replace with fwd data
    Seq(
      !aiq0ReadData.srcValid(1) -> iu_imm,                 // !srcValid -> use imm
      aiq0ReadData.srcVec(1).wb -> io.data.r(1).data,   // has write back -> use reg
    )
  )

  // iu src2 only from imm
  io.data.out.toIu.src2 := MuxCase(0.U, // Todo: replace with fwd data
    Seq(
      // Todo: fix r(2), should be pipe3 src1 data
      aiq0ReadData.srcVec(2).wb -> io.data.r(2).data,  // has write back -> use reg
    )
  )

  // Todo: figure out src1NoImm
  io.data.out.toIu.src1NoImm := MuxCase(0.U, // Todo: replace with fwd data
    Seq(
      aiq0ReadData.srcVec(1).wb -> io.data.r(1).data,  // has write back -> use reg
    )
  )

  private val iuSrcNoReadyVec = Wire(Vec(AiqConfig.NumSrcArith, Bool()))
  // src no ready when need read reg but data has not been write back
  iuSrcNoReadyVec.zipWithIndex.foreach {
    case (noReady, i) =>
      noReady := aiq0ReadData.srcValid(i) && !aiq0ReadData.srcVec(i).wb // Todo: && !fwd
  }

  pipeSrcNotReadyVec(0) := iuSrcNoReadyVec.reduce(_||_)
  io.data.out.toAiq0.readyClr.zipWithIndex.foreach {
    case (readyClr, i) =>
      readyClr := iuSrcNoReadyVec(i) && !pipeOtherLaunchFailVec(0)
  }

  //----------------------------------------------------------
  //                Output to Execution Units
  //----------------------------------------------------------

  io.data.out.toIu.iid  := aiq0ReadData.iid
  io.data.out.toIu.inst := aiq0ReadData.inst
  io.data.out.toIu.dstPreg.valid := aiq0ReadData.dstValid
  io.data.out.toIu.dstPreg.bits := aiq0ReadData.dstPreg
  io.data.out.toIu.dstVPreg.valid := aiq0ReadData.dstVValid
  io.data.out.toIu.dstVPreg.bits := aiq0ReadData.dstVreg
  io.data.out.toIu.opcode := opcode
//  io.data.out.toIu.src0
//  io.data.out.toIu.src1
//  io.data.out.toIu.src2
//  io.data.out.toIu.src1NoImm
  io.data.out.toIu.imm := DontCare // Todo: figure out and fix
  io.data.out.toIu.aluShort := aiq0ReadData.aluShort
  io.data.out.toIu.rsltSel := DontCare // Todo: fix or remove
  io.data.out.toIu.vlmul := aiq0ReadData.vlmul
  io.data.out.toIu.vsew := aiq0ReadData.vsew
  io.data.out.toIu.vl := aiq0ReadData.vl
  // output to special
  io.data.out.toIu.exceptVec := aiq0ReadData.exceptVec
  io.data.out.toIu.highHwExpt := aiq0ReadData.highHwExcept
  io.data.out.toIu.specialImm := iu_imm(19, 0)
  io.data.out.toIu.pid := aiq0ReadData.pid
  // output to cp0
  io.ctrl.out.toCp0.iid := aiq0ReadData.iid
  io.ctrl.out.toCp0.inst := aiq0ReadData.inst
  io.ctrl.out.toCp0.preg := aiq0ReadData.dstPreg
  // Todo: Had
  io.ctrl.out.toCp0.src0 := io.data.r(0).data
  io.ctrl.out.toCp0.opcode := opcode





  //==========================================================
  //                RF Execution Unit Selection
  //==========================================================
  // Todo:
  //----------------------------------------------------------
  //               Pipe0 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toCp0.sel   := funcUnit === FuncUnit.CP0 && pipeDownValidVec(0)
  io.ctrl.out.toAlu0.sel  := funcUnit === FuncUnit.ALU && pipeDownValidVec(0)

  io.ctrl.out.toCp0.gateClkSel  := funcUnit === FuncUnit.CP0 && pipeInstValidVec(0)
  io.ctrl.out.toAlu0.gateClkSel := funcUnit === FuncUnit.ALU && pipeInstValidVec(0)
  //----------------------------------------------------------
  //               Pipe1 Exectuion Unit Selection
  //----------------------------------------------------------

  //----------------------------------------------------------
  //               Pipe2 Exectuion Unit Selection
  //----------------------------------------------------------


  //==========================================================
  //                    Pipe0 Data Path
  //==========================================================

  //----------------------------------------------------------
  //                   Pipeline Registers
  //----------------------------------------------------------
  io.data.out.toAiq0.launchEntryOH := RegEnable(aiq0_data.issueEntryOH, aiq0_data.issueEn)


  //==========================================================
  //                   Performance Monitor
  //==========================================================
  private val pipeInstValid : Bool = pipeInstValidVec.reduce(_||_)
  private val pipeInstValidVecFF : Vec[Bool] = RegEnable(pipeInstValidVec, io.ctrl.in.fromHpcp.iduCntEn)
  io.ctrl.out.toHpcp.instValid := RegNext(pipeInstValid)
  io.ctrl.out.toHpcp.pipeVec.zipWithIndex.foreach {
    case (signal, i) => signal.instValid := pipeInstValidVecFF(i)
  }

  //----------------------------------------------------------
  //                    RF inst valid
  //----------------------------------------------------------


  io.ctrl.out.toSd := DontCare
  io.ctrl.out.toSt := DontCare
  io.ctrl.out.toBju := DontCare
  io.ctrl.out.toAlu1 := DontCare
  io.ctrl.out.toLu := DontCare
  io.ctrl.out.toSpecial := DontCare
  io.ctrl.out.toVfpu0 := DontCare
  io.ctrl.out.toVfpu1 := DontCare
  io.ctrl.out.toMul := DontCare
  io.ctrl.out.toDiv := DontCare

  io.data.out.toAiq1 := DontCare
  io.data.out.toBiq := DontCare
  io.data.out.toVfiq0 := DontCare
  io.data.out.toVfiq1 := DontCare
  io.data.out.toLsiq0 := DontCare
  io.data.out.toLsiq1 := DontCare
  io.data.out.toSdiq := DontCare


}
