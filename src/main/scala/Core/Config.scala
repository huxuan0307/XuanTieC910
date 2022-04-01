package Core

import chisel3._
import chisel3.util._

object FuncOpType {
  def width = 7.W
  def uwidth = UInt(width)
}

object FuncType {
  def typeSize = 6
  def alu = 0.U
  def lsu = 1.U
  def mdu = 2.U
  def csr = 3.U
  def mou = 4.U
  def bru = 5.U
  def width = log2Up(typeSize).W
  def uwidth = UInt(width)
}

trait IntConfig {
  def NumAlu = 2
  def NumLu = 1 // Load
  def NumSu = 1 // Store
  def NumMu = 1 // Mul
  def NumDu = 1 // Div
  def NumVfUnit = 2 // Vector and Float
  def NumFu : Int = NumAlu + NumLu + NumSu +
    NumMu + NumDu + NumVfUnit
  def NumFuHasDstReg : Int = NumAlu + NumLu + NumMu + NumDu + NumVfUnit
  def NumLogicRegs = 32
  def NumPhysicRegs = 96
  def OpcodeBits = 32
  def NumInstructionId = 128
  def NumLogicRegsBits : Int = log2Up(NumLogicRegs)
  def NumPhysicRegsBits : Int = log2Up(NumPhysicRegs)
  def InstructionIdWidth : Int = log2Up(NumInstructionId)
  def XLEN = 64
}

trait ROBConfig {
  def NumRobEntry     = 64
  def NumRobEntryBits : Int = log2Up(NumRobEntry)
  def RobPtrWidth     : Int = NumRobEntry
  def RobPtrNum       = 4
  def RobReadPtrNum   = 6
  def NumRobReadEntry = 3
  def NumCreateEntry  = 4
  def NumCommitEntry  = 3
  def NumRetireEntry  = 3
  def NumPopEntry     = 3
  def RobPcOffsetBits = 3
  // Todo: Num of complete
  def NumCmpltBits    = 2
}

trait PipelineConfig {
  def NumIuPipe = 2
  def NumLsuPipe = 1
  def NumPipeline = 7
}

trait AddrConfig {
  def PcWidth = 39
}

trait ExceptionConfig {
  def ExceptionVecWidth = 5
  def InterruptVecWidth = 5
  def MtvalWidth = 32
}

trait VectorUnitConfig {
  def VlmaxBits = 8
  def VlmulBits = 2 // TODO: should be 2 or 3? @ct_cp0_top.v @293  [1  :0]  rtu_cp0_vsetvl_vlmul;
  def VsewBits = 3
  def VstartBits = 7
  def VregNum = 64
  def VregNumBits : Int = log2Up(VregNum)
}

trait IUConfig {
  def XLEN = 64
  def MPPWidth = 2
  def PcFifoLen = 32
  def PcFifoAddr: Int = log2Up(PcFifoLen)
  def PcOffsetWidth = 21
  def IuPipeNum = 3
}

trait LsuConfig{
  def PA_WIDTH = 40
  def FENCE_MODE_WIDTH = 4
  def INST_CODE_WIDTH = 32
  def INST_MODE_WIDTH = 2
  def INST_SIZE_WIDTH = 2
  def INST_TYPE_WIDTH = 2
  def OFFSET_WIDTH = 12
  def LSU_PC_WIDTH = 15 //@ ct_lst_st_ag.v  534  parameter PC_LEN = 15;
  def SDIQ_ENYTY_ADDR = 12
  def SHITF_WIDTH = 4

  def ADDR_PA_WIDTH = 28

  def WAIT_OLD_WIDTH = 12
  def ACCESS_SIZE_CHOOSE = 3
  def BYTES_ACCESS_WIDTH = 16
  def ROT_SEL_WIDTH = 4
  def LSIQ_ENTRY = 12
  def VPN_WIDTH = 28

  def SNOOP_ID_WIDTH = 6

  def SDID_WIDTH = log2Up(LSIQ_ENTRY)

  def DCACHE_DIRTY_ARRAY_WITDH = 7
  def DCACHE_TAG_ARRAY_WITDH   = 52

  def PREG_SIGN_SEL = 4
  def VREG_SIGN_SEL = 2
  def DATA_UPDATE_PATH_WIDTH = 5
  def VMB_ENTRY = 8
}
object LsuAccessSize extends LsuConfig{
  def byte:  UInt = 0.U(ACCESS_SIZE_CHOOSE.W)
  def half:  UInt = 1.U(ACCESS_SIZE_CHOOSE.W)
  def word:  UInt = 2.U(ACCESS_SIZE_CHOOSE.W)
  def dword: UInt = 3.U(ACCESS_SIZE_CHOOSE.W)
  def qword: UInt = 4.U(ACCESS_SIZE_CHOOSE.W)
}
trait DCacheConfig {
  def PA_WIDTH = 40
  def TOTAL_SIZE = 64 //Kb
  def WAYS = 2
  def LINE_SIZE = 64 // byte
  def SET: Int = TOTAL_SIZE * 1024 / LINE_SIZE / WAYS // 512
  def OFFSET_WIDTH: Int = log2Up(LINE_SIZE) // 6
  def INDEX_WIDTH: Int = log2Up(SET) // 9
  def TAG_WIDTH: Int = PA_WIDTH - OFFSET_WIDTH - INDEX_WIDTH // 25
}
trait Cp0Config {
  def APB_BASE_WIDTH = 40
  def BIU_CP0_RDATA = 128
  def BIU_CP0_RVBA = 40
  def FSER_ACC_UPDATE_WITDH = 7
  def CACHE_READ_DATA_WITDTH = 128
  def CSR_OPCODE_WIDTH = 32
  def CSR_ADDR_WIDTH = 12
  def CSR_UIMM_WIDTH = 5
  def CSR_OTHERS_WIDTH: Int = CSR_OPCODE_WIDTH-CSR_ADDR_WIDTH-CSR_UIMM_WIDTH
}


object MDUOpType {
  def mul    = "b0000".U
  def mulh   = "b0001".U
  def mulhsu = "b0010".U
  def mulhu  = "b0011".U
  def div    = "b0100".U
  def divu   = "b0101".U
  def rem    = "b0110".U
  def remu   = "b0111".U

  def mulw   = "b1000".U
  def divw   = "b1100".U
  def divuw  = "b1101".U
  def remw   = "b1110".U
  def remuw  = "b1111".U

  def isDiv(op: UInt) = op(2)
  def isDivSign(op: UInt) = isDiv(op) && !op(0)
  def isW(op: UInt) = op(3)
  def isRem(op: UInt) = op(2) && op(1)
}
object IntConfig extends IntConfig
object ROBConfig extends ROBConfig
object PipelineConfig extends PipelineConfig
object AddrConfig extends AddrConfig
object ExceptionConfig extends ExceptionConfig
object VectorUnitConfig extends VectorUnitConfig
object IUConfig extends IUConfig
object LsuConfig extends LsuConfig
object DCacheConfig extends DCacheConfig