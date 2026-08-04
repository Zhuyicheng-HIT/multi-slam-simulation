param(
    [string]$Port = "COM17",
    [int]$BaudRate = 115200,
    [string]$ListenAddress = "0.0.0.0",
    [int]$TcpPort = 5764
)

$ErrorActionPreference = "Stop"
$serial = $null
$listener = $null
$client = $null
$stream = $null
$totalBytes = 0L
$lastReport = [System.Diagnostics.Stopwatch]::StartNew()

try {
    $serial = [System.IO.Ports.SerialPort]::new(
        $Port,
        $BaudRate,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One
    )
    $serial.Handshake = [System.IO.Ports.Handshake]::None
    $serial.DtrEnable = $false
    $serial.RtsEnable = $false
    $serial.ReadTimeout = 100
    $serial.ReadBufferSize = 65536
    $serial.Open()
    $serial.DiscardInBuffer()

    $address = [System.Net.IPAddress]::Parse($ListenAddress)
    $listener = [System.Net.Sockets.TcpListener]::new($address, $TcpPort)
    $listener.Start()
    Write-Host "MTF-01 read-only bridge: $Port @ $BaudRate 8N1 -> ${ListenAddress}:$TcpPort"
    Write-Host "No bytes are written to the sensor. Waiting for a WSL client..."

    while ($true) {
        if ($null -eq $client -and $listener.Pending()) {
            $client = $listener.AcceptTcpClient()
            $client.NoDelay = $true
            $stream = $client.GetStream()
            Write-Host "WSL client connected: $($client.Client.RemoteEndPoint)"
        }

        $available = $serial.BytesToRead
        if ($available -gt 0) {
            $buffer = [byte[]]::new([Math]::Min($available, 8192))
            $read = $serial.Read($buffer, 0, $buffer.Length)
            $totalBytes += $read
            if ($null -ne $stream) {
                try {
                    $stream.Write($buffer, 0, $read)
                }
                catch {
                    Write-Warning "WSL client disconnected: $($_.Exception.Message)"
                    $stream.Dispose()
                    $client.Dispose()
                    $stream = $null
                    $client = $null
                }
            }
        }
        else {
            Start-Sleep -Milliseconds 2
        }

        if ($lastReport.Elapsed.TotalSeconds -ge 5.0) {
            Write-Host ("serial_bytes={0} client_connected={1}" -f $totalBytes, ($null -ne $client))
            $lastReport.Restart()
        }
    }
}
finally {
    if ($null -ne $stream) { $stream.Dispose() }
    if ($null -ne $client) { $client.Dispose() }
    if ($null -ne $listener) { $listener.Stop() }
    if ($null -ne $serial) {
        if ($serial.IsOpen) { $serial.Close() }
        $serial.Dispose()
    }
}
