package Core.IDU.IS

import Core.IntConfig._
import chisel3._
import chisel3.util._

trait SdiqConfig {
  def NumSrcSd = 2
  def NumSrcSdX = 1
  def NumSdiqEntry = 12
  def NumSdiqCreatePort = 2
}

object SdiqConfig extends SdiqConfig

class SdiqEntryData extends Bundle with SdiqConfig {
  val load : Bool = Bool()
  val stAddr1InStq : Bool = Bool()
  val stAddr0InStq : Bool = Bool()
  // 0: data 0 valid, 1: data 1 valid
  val stData1Valid : Bool = Bool()
  val unalign : Bool = Bool()
  val src0 = new DepRegEntryData
  val srcV0 = new DepRegEntryData // Todo: Vreg
  val src0Valid : Bool = Bool()
  val srcV0Valid : Bool = Bool()
}

class SdiqEntryInput
  extends IqEntryInput(SdiqConfig.NumSdiqEntry, SdiqConfig.NumSrcSd, SdiqConfig.NumSrcSdX)
    with IqEntryHasVectorInputBundle
    with SdiqConfig with DepRegEntryConfig {
  val create = new Bundle {
    val ageVec : Vec[Bool] = Vec(NumSdiqEntry - 1, Bool())
    val data = new SdiqEntryData
    val dpEn : Bool = Bool()
    val en : Bool = Bool()
    val gateClkEn : Bool = Bool()
  }
  val freezeClrEx1 : Bool = Bool()
  val fromRf = new Bundle {
    val freezeClr : Bool = Bool()
    val stAddr1Valid : Bool = Bool()
    val stData1Valid : Bool = Bool()
    val stAddrReadyClear : Bool = Bool()
  }
  val stAddrReadySet : Bool = Bool()
  val stAddrStqCreate : Bool = Bool()

  val fromLsu = new Bundle {
    val stAddr1Valid : Bool = Bool()
    val stAddrUnalign : Bool = Bool()
  }
}

class SdiqEntryOutput
  extends IqEntryOutput(SdiqConfig.NumSdiqEntry)
    with SdiqConfig with DepRegEntryConfig {
  val readData = new SdiqEntryData
  val src0PregOH : UInt = UInt(NumPhysicRegs.W)
  val srcFPregOH : UInt = UInt(NumFPregs.W)
  val srcVPregOH : UInt = UInt(NumVPregs.W)
}

class SdiqEntryIO extends Bundle {
  val in : SdiqEntryInput = Flipped(Output(new SdiqEntryInput))
  val out : SdiqEntryOutput = Output(new SdiqEntryOutput)
}

class SdiqEntry extends Module with SdiqConfig {
  val io : SdiqEntryIO = IO(new SdiqEntryIO)

  private val create = io.in.create
  private val rtu = io.in.fromRtu
  private val lsu = io.in.fromLsu
  private val rf = io.in.fromRf

  private val srcEntries = Seq.fill(NumSrcSdX)(Module(new DepRegEntry))

  /**
   * Regs
   */

  private val valid = RegInit(false.B)
  private val ageVec = RegInit(VecInit(Seq.fill(this.NumSdiqEntry - 1)(false.B)))
  private val freeze = RegInit(false.B)
  private val stAddr0Ready = RegInit(false.B)
  private val stAddr1Ready = RegInit(false.B)
  private val data = RegInit(0.U.asTypeOf(Output(new SdiqEntryData)))

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================

  private val entryClkEn = create.gateClkEn || valid

  // Todo: gated clk

  private val gateClkEntryValid = valid

  //==========================================================
  //                  Create and Read Bus
  //==========================================================
  //SDIQ is flush by backend rtu_yy_xx_flush, not frontend or IS

  //==========================================================
  //                      Entry Valid
  //==========================================================
  when(rtu.flush.be) {
    valid := false.B
  }.elsewhen(create.en) {
    valid := true.B
  }.elsewhen(io.in.popCurEntry && io.in.popValid) {
    valid := false.B
  }
  io.out.valid := valid

  //==========================================================
  //                        Freeze
  //==========================================================

  when(create.en) {
    //SDIQ cannot bypass, create in frz is always 0
    freeze := false.B
  }.elsewhen(io.in.fromRf.freezeClr || io.in.freezeClrEx1) {
    freeze := false.B
  }.elsewhen(io.in.issueEn) {
    freeze := true.B
  }
  io.out.validWithoutFreeze := valid && !freeze

  //==========================================================
  //                       Age Vector
  //==========================================================

  when(create.en) {
    ageVec := create.ageVec
  }.elsewhen(io.in.popValid) {
    ageVec := ageVec.zip(io.in.popOtherEntry).map {
      case (age, popOther) => age & !popOther
    }
  }
  io.out.ageVec := ageVec

  //==========================================================
  //                 Instruction Information
  //==========================================================

  when(create.dpEn) {
    data.src0Valid := create.data.src0Valid
    data.srcV0Valid := create.data.srcV0Valid
    data.load := create.data.load
  }

  //==========================================================
  //             Store Address Instruction Ready
  //==========================================================
  //----------------------------------------------------------
  //                    Staddr 0 Ready
  //----------------------------------------------------------

  private val stAddr0ReadyClear = rf.freezeClr &&
    rf.stAddrReadyClear && !rf.stData1Valid
  private val stAddr0ReadySet = data.stAddr0InStq ||
    io.in.stAddrReadySet && !rf.stAddr1Valid

  when(create.en) {
    stAddr0Ready := false.B
  }.elsewhen(stAddr0ReadyClear) {
    stAddr0Ready := false.B
  }.elsewhen(stAddr0ReadySet) {
    stAddr0Ready := true.B
  }

  //----------------------------------------------------------
  //                    Staddr 1 Ready
  //----------------------------------------------------------

  private val stAddr1ReadyClear = rf.freezeClr &&
    rf.stAddrReadyClear && rf.stData1Valid
  private val stAddr1ReadySet = data.stAddr1InStq ||
    io.in.stAddrReadySet && rf.stAddr1Valid

  when(create.en) {
    stAddr1Ready := false.B
  }.elsewhen(stAddr1ReadyClear) {
    stAddr1Ready := false.B
  }.elsewhen(stAddr1ReadySet) {
    stAddr1Ready := true.B
  }

  //----------------------------------------------------------
  //                 Staddr 0 in Store Queue
  //----------------------------------------------------------

  when(create.en) {
    data.stAddr0InStq := false.B
    data.unalign := false.B
  }.elsewhen(io.in.stAddrStqCreate && !lsu.stAddr1Valid) {
    data.stAddr0InStq := true.B
    data.unalign := lsu.stAddrUnalign
  }

  //----------------------------------------------------------
  //                 Staddr 1 in Store Queue
  //----------------------------------------------------------

  when(create.en) {
    data.stAddr1InStq := false.B
  }.elsewhen(io.in.stAddrStqCreate && lsu.stAddr1Valid) {
    data.stAddr1InStq := true.B
  }

  //==========================================================
  //                  Store Data 1 Valid
  //==========================================================
  //indicate the current entry is unalign stdata inst 1

  when(create.en) {
    data.stData1Valid := false.B
  }.elsewhen(io.in.freezeClrEx1) {
    data.stData1Valid := true.B
  }

  //==========================================================
  //              Source Dependency Information
  //==========================================================
  private val src0ReadyClear = rf.freezeClr && io.in.readyClr(0)
  private val srcV0ReadyClear = rf.freezeClr && io.in.readyClr(1)

  //------------------------source 0--------------------------
  private val createSrcGateClkEn = create.gateClkEn && create.data.src0Valid
  private val srcReadData = Wire(new DepRegEntryReadData)

  srcEntries.zipWithIndex.foreach {
    case (entry, i) =>
      val in = entry.io.in
      val out = entry.io.out
      in.fwdValid       := io.in.fwdValid(i)
      in.fromCp0        := io.in.fromCp0
      in.fuDstPreg      := io.in.fuDstPreg
      in.wbPreg         := io.in.wbPreg
      in.loadPreg       := io.in.loadPreg
      in.flush.is       := rtu.flush.be // Todo: Figure out why use flush backend signal
      in.flush.fe       := rtu.flush.be
      in.flush.be       := rtu.flush.be
      in.createData     := create.data.src0
      in.gateClkIdxWen  := createSrcGateClkEn
      in.gateClkWen     := create.gateClkEn
      in.readyClear     := src0ReadyClear
      in.wen            := create.dpEn

      srcReadData       := out.readData
  }

  // Todo: src entry for vector
  io.out.readData.srcV0 := DontCare
  io.out.srcVPregOH := DontCare
  io.out.srcFPregOH := DontCare

  io.out.readData.src0.wb := srcReadData.wb
  io.out.readData.src0.preg := srcReadData.preg
  io.out.readData.src0.ready := false.B
  io.out.readData.src0.lsuMatch := false.B

  io.out.readData.src0Valid := data.src0Valid
  io.out.readData.srcV0Valid := data.srcV0Valid
  io.out.readData.load := data.load

  io.out.readData.unalign := data.unalign ||
    io.in.stAddrStqCreate && !lsu.stAddr1Valid && lsu.stAddrUnalign
  io.out.readData.stAddr0InStq := data.stAddr0InStq ||
    io.in.stAddrStqCreate && !lsu.stAddr1Valid
  io.out.readData.stAddr1InStq := data.stAddr1InStq ||
    io.in.stAddrStqCreate && lsu.stAddr1Valid

  io.out.readData.stData1Valid := data.stData1Valid

  io.out.src0PregOH := Mux (
    valid && data.src0Valid,
    UIntToOH(srcReadData.preg),
    0.U
  )

  io.out.ready := valid && !freeze &&
    srcReadData.readyForIssue &&
    true.B && // Todo: add src v0 ready for issue
    (data.load ||
      !data.stData1Valid && stAddr0Ready ||
      data.stData1Valid && stAddr1Ready)
}
