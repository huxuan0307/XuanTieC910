package Core.LSU

trait LSUConfig {
  def PA_WIDTH = 40

  def LSIQ_ENTRY  = 12
  def LQ_ENTRY    = 16
  def SQ_ENTRY    = 12
  def PC_LEN      = 15

  def VB_DATA_ENTRY = 3
  def WMB_ENTRY     = 8
  def VMB_ENTRY     = 8

  def BYTE        = "b00"
  def HALF        = "b01"
  def WORD        = "b10"
  def DWORD       = "b11"

  def BIU_R_NORM_ID_T     = 1
  def BIU_R_CTC_ID        = 28
  def BIU_B_NC_ID         = 24
  def BIU_B_SO_ID         = 29
  def BIU_B_NC_ATOM_ID    = 30
  def BIU_B_SYNC_FENCE_ID = 31

  def EXOKAY              = 1
}

object LSUConfig extends LSUConfig
