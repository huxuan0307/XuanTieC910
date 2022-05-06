package Core
import chisel3._
import chisel3.util._

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
  def IidWidth        = NumRobEntryBits+1
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

trait FuTypeConfig {
  def ALU = "b0000000001"

  def BJU = "b0000000010"

  def MULT = "b0000000100"

  def DIV = "b0000001000"

  def LSU_P5 = "b0000110000"

  def LSU = "b0000010000"

  def PIPE67 = "b0001000000"

  def PIPE6 = "b0010000000"

  def PIPE7 = "b0100000000"

  def SPECIAL = "b1000000000"
}
trait IUConfig {
  def XLEN = 64
  def MPPWidth = 2
  def PcFifoLen = 32
  def PcFifoAddr: Int = log2Up(PcFifoLen)
  def PcOffsetWidth = 21
  def IuPipeNum = 3
}

trait BIUConfig {
  def BIU_R_NORM_ID_T     = 1.U(2.W)
  def BIU_R_CTC_ID        = 28.U(5.W)
  def BIU_B_NC_ID         = 24.U(5.W)
  def BIU_B_SO_ID         = 29.U(5.W)
  def BIU_B_NC_ATOM_ID    = 30.U(5.W)
  def BIU_B_SYNC_FENCE_ID = 31.U(5.W)

  def BIU_R_NC_ID         = 24.U(5.W)
  def BIU_R_SO_ID         = 29.U(5.W)
  def BIU_R_NC_ATOM_ID    = 30.U(5.W)
  def BIU_R_SYNC_FENCE_ID = 31.U(5.W)

  def OKAY   = 0.U(2.W)
  def EXOKAY = 1.U(2.W)
  def SLVERR = 2.U(2.W)
  def DECERR = 3.U(2.W)
}

trait LsuConfig{
  def PA_WIDTH = 40
  def VPN_WIDTH = 28
  def PPN_WIDTH = 28
  def FENCE_MODE_WIDTH = 4
  def INST_CODE_WIDTH = 32
  def INST_MODE_WIDTH = 2
  def INST_SIZE_WIDTH = 2
  def INST_TYPE_WIDTH = 2

  def LSU_PC_WIDTH = 15 //@ ct_lst_st_ag.v  534  parameter PC_LEN = 15;

  def SHITF_WIDTH = 4

  def ADDR_PA_WIDTH = 28

  def WAIT_OLD_WIDTH = 12
  def ACCESS_SIZE_CHOOSE = 3
  def BYTES_ACCESS_WIDTH = 16
  def ROT_SEL_WIDTH = 4
  def ROT_SEL_WIDTH_8 = 8

  def LSIQ_ENTRY  = 12
  def LQ_ENTRY    = 16
  def SQ_ENTRY    = 12
  def VB_DATA_ENTRY = 3
  def WMB_ENTRY     = 8
  def VMB_ENTRY     = 8
  def RB_ENTRY = 8

  def SNOOP_ID_WIDTH = 6
  def SDID_WIDTH = log2Up(SQ_ENTRY)

  //def DCACHE_DIRTY_ARRAY_WITDH = 7
  //def DCACHE_TAG_ARRAY_WITDH   = 52

  def PREG_SIGN_SEL = 4
  def VREG_SIGN_SEL = 2
  def DATA_UPDATE_PATH_WIDTH = 5

  def BYTE        = "b00"
  def HALF        = "b01"
  def WORD        = "b10"
  def DWORD       = "b11"

  def CACHE_DIST_SELECT_ADDR = 4
  def CACHE_DIST_SELECT = log2Up(CACHE_DIST_SELECT_ADDR)
}
object LsuAccessSize extends LsuConfig{
  def byte:  UInt = 0.U(ACCESS_SIZE_CHOOSE.W)
  def half:  UInt = 1.U(ACCESS_SIZE_CHOOSE.W)
  def word:  UInt = 2.U(ACCESS_SIZE_CHOOSE.W)
  def dword: UInt = 3.U(ACCESS_SIZE_CHOOSE.W)
  def qword: UInt = 4.U(ACCESS_SIZE_CHOOSE.W)
}
trait DCacheConfig {
  def TOTAL_SIZE = 64 //Kb
  def WAYS = 2
  def LINE_SIZE = 64 // byte
  def SET: Int = TOTAL_SIZE * 1024 / LINE_SIZE / WAYS // 512
  def OFFSET_WIDTH: Int = log2Up(LINE_SIZE) // 6
  def INDEX_WIDTH: Int = log2Up(SET) // 9
  def TAG_WIDTH: Int = LsuConfig.PA_WIDTH - OFFSET_WIDTH - INDEX_WIDTH // 25
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
object FuTypeConfig extends FuTypeConfig
object IUConfig extends IUConfig
object LsuConfig extends LsuConfig
object DCacheConfig extends DCacheConfig
