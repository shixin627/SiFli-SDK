$out = "C:\skaiwalk\SiFli-SDK\example\get-started\dualcore\project\hcpu\_watch_console.log"
while ($true) {
    try {
        $port = New-Object System.IO.Ports.SerialPort COM12,1000000,None,8,one
        $port.ReadTimeout = 500
        $port.Open()
        Add-Content -Path $out -Value "--- reconnected $(Get-Date -Format 'HH:mm:ss') ---"
        while ($true) {
            try {
                $line = $port.ReadLine()
                Add-Content -Path $out -Value $line
            } catch [System.TimeoutException] {
                continue
            }
        }
    } catch {
        Add-Content -Path $out -Value "--- reader error, retrying: $_ ---"
        if ($port -and $port.IsOpen) { try { $port.Close() } catch {} }
        Start-Sleep -Seconds 2
    }
}
