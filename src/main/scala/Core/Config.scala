package Core

//import Core.LsuConfig.PA_WIDTH
import chisel3.util._
import chisel3._

object GlobalConfig {
  def DifftestEnable: Boolean = true
  def NumFoldMax = 3
}

object FuncOpType {
  def width  = 7.W
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
  def NumFPregs = 64
  def NumVPregs = 64
  def NumEregs = 32
  def NumPhysicRegs = 96
  def InstBits = 32
  def NumInstructionId = 128
  def NumLogicRegsBits : Int = log2Up(NumLogicRegs)
  def NumFPregsBits : Int = log2Up(NumFPregs)
  def NumVPregsBits : Int = log2Up(NumVPregs)
  def NumEregsBits : Int = log2Up(NumEregs)
  def NumPhysicRegsBits : Int = log2Up(NumPhysicRegs)
  def InstructionIdWidth : Int = log2Up(NumInstructionId)
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
  def NumLsuPipe = 2
  def NumLuPipe = 1
  def NumSuPipe = 1
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



trait FuTypeConfig {
  def ALU      = "b0000000001"
  def BJU      = "b0000000010"
  def MULT     = "b0000000100"
  def DIV      = "b0000001000"
  def LSU_P5   = "b0000110000"
  def LSU      = "b0000010000"
  def PIPE67   = "b0001000000"
  def PIPE6    = "b0010000000"
  def PIPE7    = "b0100000000"
  def SPECIAL  = "b1000000000"
}

trait IUConfig {
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

  def pa_widthBits = 40
  def barBits = 2
  def burstBits = 2
  def cacheBits = 4
  def domainBits = 2
  def idBits = 5
  def lenBits = 2
  def protBits = 3
  def sizeBits = 3
  def ar_snoopBits = 4
  def aw_snoopBits = 3
  def ar_userBits = 3

  def dataBits = 128
  def strbBits = dataBits/8


  def qosBits = 4
  def respBits = 2
  def regionBits = 4
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
  def VB_ADDR_ENTRY = 2
  def WMB_ENTRY     = 8
  def VMB_ENTRY     = 8
  def RB_ENTRY = 8
  def SNQ_ENTRY     = 6

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
  // (Tag, Index, Offset)
  def PFU_ENTRY = 9
  def PFU_IDX = log2Up(PFU_ENTRY)

  def LFB_ADDR_ENTRY = 8
  def LFB_DATA_ENTRY = 2
  def LFB_ID_WIDTH = 3

}


object BiuID {
  def BIU_LFB_ID_T = "b00".U
  def BIU_VB_ID_T = "b000".U
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
object FuTypeConfig extends FuTypeConfig

abstract class CoreBundle extends Bundle with Config {}

trait Config {
  val PcStart = "h80000000"
  def XLEN = 64
  val VAddrBits = 39
  val PAddrBits = 40

  //IFU
  val uBTBSize = 16
  val pre_array_data_size = 32
  val ghr_size = 22
  val rtu_ras = 6
  val ifu_ras = 12
  val IBufSize = 32

  //CtrlBlock
  val RobSize = 128
  val NRPhyRegs = 64
  //LSU
  val LSQueueSize = 32
  val StoreBufferSize = 16

  //DCache
  val DCacheSize    = 64 //KB
  val CacheLineBits = 512
  val CacheLineSize = CacheLineBits/8
  val DCacheWays    = 2
  val DCacheSets    = DCacheSize*1024/CacheLineSize/DCacheWays //512
  val OffsetBits    = log2Up(CacheLineSize) //6
  val IndexBits     = log2Up(DCacheSets) //9
  val DCacheTagBits = PAddrBits - 12
  val DCRefillBits  = 64
  val RefillCnt     = CacheLineBits/DCRefillBits
}

object Config extends Config{}

object LSUOpType {
  def lb   = "b0000000".U
  def lh   = "b0000001".U
  def lw   = "b0000010".U
  def ld   = "b0000011".U
  def lbu  = "b0000100".U
  def lhu  = "b0000101".U
  def lwu  = "b0000110".U
  def sb   = "b0001000".U
  def sh   = "b0001001".U
  def sw   = "b0001010".U
  def sd   = "b0001011".U

  def lr   = "b0100000".U
  def sc   = "b0100001".U

  def isStore(func: UInt): Bool = func(3)
  def isLoad(func: UInt): Bool = !isStore(func)
  def isLR(func: UInt): Bool = func === lr
  def isSC(func: UInt): Bool = func === sc
  def needMemRead(func: UInt): Bool  = isLoad(func) || isLR(func)
  def needMemWrite(func: UInt): Bool = isStore(func) || isSC(func)
}
