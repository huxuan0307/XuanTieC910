package Core.RF

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Core.PipelineConfig._
import Core.IntConfig._

trait PrfConfig {
  def NumPregReadPort = 11
  def NumPregWritePort = 3
}

class PrfFromCp0Bundle extends Bundle {
  val iduIcgEn : Bool = Bool()
  val yyClkEn : Bool = Bool()
}

class PrfFromPadBundle extends Bundle {
  val yyIcgScanEn : Bool = Bool()
}

class PrfFromRtuBundle extends Bundle {
  val yyXxDebugOn : Bool = Bool()
}

class PrfToHadBundle extends Bundle {
  val wb = new Bundle {
    val data : UInt = UInt(XLEN.W)
    val valid : Bool = Bool()
  }
}

/**
 * Elements has different direction
 */
class PrfReadBundle extends Bundle {
  val preg : UInt = Input(UInt(NumPhysicRegsBits.W))
  val data : UInt = Output(UInt(XLEN.W))
}

class PrfWriteBundle extends Bundle {
  val en : Bool = Bool()
  val preg : UInt = UInt(NumPhysicRegsBits.W)
  val data : UInt = UInt(XLEN.W)
}

class PrfInput extends Bundle {
  val fromCp0 = new PrfFromCp0Bundle
  val fromPad = new PrfFromPadBundle
  val fromRtu = new PrfFromRtuBundle
}

class PrfOutput extends Bundle {
  val toHad = new PrfToHadBundle
}

class PrfIO extends Bundle with PrfConfig {
  val in : PrfInput = Input(new PrfInput)
  val out : PrfOutput = Output(new PrfOutput)
  val r : Vec[PrfReadBundle] = Vec(NumPregReadPort, new PrfReadBundle)
  val w : Vec[PrfWriteBundle] = Input(Vec(NumPregWritePort, new PrfWriteBundle))
}

/**
 * Physical regfile
 */
class Prf extends Module {
  val io : PrfIO = IO(new PrfIO)

  /**
   * Input rename
   */
  val wenVec    : Vec[Bool] = VecInit(io.w.map(_.en))
  val wdataVec  : Vec[UInt] = VecInit(io.w.map(_.data))
  val wPregVec  : Vec[UInt] = VecInit(io.w.map(_.preg))
  val rPregVec  : Vec[UInt] = VecInit(io.r.map(_.preg))
  /**
   * Regs
   */
  private val data = RegInit(VecInit(Seq.fill(NumPhysicRegs)(0.U(XLEN.W))))

  val wen     : Bool = wenVec.reduce(_||_)
  val wenCnt  : UInt = wenVec.count(b => b)
  // Only one wen high is permitted
  val fault   : Bool = !(wenCnt === 0.U || wenCnt === 1.U)
  val wdata   : UInt = wdataVec(OHToUInt(wenVec))
  val wPreg   : UInt = wPregVec(OHToUInt(wenVec))

  // Todo: gated clk

  when(!fault && wen) {
    when(wPreg =/= 0.U) {
      data(wPreg) := wdata
    }
  }

  io.r.zip(rPregVec).foreach{
    case (r, preg) => r.data := data(preg)
  }

  io.out.toHad.wb.valid := io.in.fromRtu.yyXxDebugOn && wen
  io.out.toHad.wb.data  := Mux(wen, wdata, 0.U)
}
