# auto_send.ps1
# Sends STX (0x02) immediately and every X minutes; prints ALL incoming bytes to the console.

# ==== SETTINGS ====
$port = "COM5"      # change to your COM port
$baud = 57600       # change to your baud rate
$showHex = $true    # print hex view
$showAscii = $true  # print ASCII view (printables shown, others as '.')

# ==== OPEN SERIAL PORT ====
$sp = New-Object System.IO.Ports.SerialPort $port,$baud,'None',8,1
$sp.Handshake   = [System.IO.Ports.Handshake]::None
$sp.DtrEnable   = $true    # try $false if your device dislikes it
$sp.RtsEnable   = $true    # try $false if your device dislikes it
$sp.ReadTimeout = 50
$sp.Encoding    = [System.Text.Encoding]::GetEncoding(28591) # 1:1 bytes->chars
$sp.Open()

Write-Host "Opened $port at $baud. Printing RX to console. Ctrl+C to stop."
# send once right away
$sp.BaseStream.WriteByte(0x02)
$sp.BaseStream.Flush()
Write-Host ("[{0}] TX: STX (0x02)" -f (Get-Date).ToString("HH:mm:ss"))

# ==== HELPERS ====
function BytesToAscii([byte[]]$bytes, [int]$count) {
  $sb = New-Object System.Text.StringBuilder
  for ($i=0; $i -lt $count; $i++) {
    $b = $bytes[$i]
    if ($b -ge 32 -and $b -le 126) { [void]$sb.Append([char]$b) }
    elseif ($b -in 9,10,13)       { [void]$sb.Append(" ") }
    else                          { [void]$sb.Append(".") }
  }
  $sb.ToString()
}

# ==== MAIN LOOP ====
$buf = New-Object byte[] 4096
$nextTx = (Get-Date).AddMinutes(10)

try {
  while ($true) {
    # TX timer
    if ((Get-Date) -ge $nextTx) {
      $sp.BaseStream.WriteByte(0x02)
      $sp.BaseStream.Flush()
      Write-Host ("[{0}] TX: STX (0x02)" -f (Get-Date).ToString("HH:mm:ss"))
      $nextTx = (Get-Date).AddMinutes(10)
    }

    # RX polling
    $avail = $sp.BytesToRead
    if ($avail -gt 0) {
      $count = $sp.BaseStream.Read($buf, 0, [Math]::Min($buf.Length, $avail))
      $ts = (Get-Date).ToString("HH:mm:ss")

      if ($showAscii) {
        $ascii = BytesToAscii $buf $count
        Write-Host ("[{0}] RX {1}B ASCII: {2}" -f $ts, $count, $ascii)
      }
      if ($showHex) {
        $hex = -join ($buf[0..($count-1)] | ForEach-Object { $_.ToString("X2") + ' ' })
        Write-Host ("[{0}] RX HEX: {1}" -f $ts, $hex)
      }
    } else {
      Start-Sleep -Milliseconds 20
    }
  }
}
finally {
  if ($sp -and $sp.IsOpen) { $sp.Close(); $sp.Dispose() }
  Write-Host "Closed $port."
}
