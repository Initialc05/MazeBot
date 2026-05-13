$ErrorActionPreference = 'Stop'

$CLI = 'D:\Repository\Embedded Development\Arduino\arduino-cli_1.4.1\arduino-cli.exe'
$Sketch = 'D:\QMUL Modules\Y3_S2\Microprocessor Systems Design\EoM\MazeBot\DnB\MazeBot'
$Libraries = 'D:\Repository\Embedded Development\Arduino\libraries'
$FQBN = 'STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_F446RE,upload_method=swdMethod'
$BuildPath = Join-Path $env:TEMP 'mazebot_build'

function Wait-AndExit([int]$Code) {
    Write-Host ''
    Write-Host 'Press any key to exit...'
    $null = $Host.UI.RawUI.ReadKey('NoEcho,IncludeKeyDown')
    exit $Code
}

if (-not (Test-Path $CLI)) {
    Write-Host 'arduino-cli not found:' -ForegroundColor Red
    Write-Host $CLI -ForegroundColor Red
    Wait-AndExit 1
}

try {
    Write-Host '[1/2] Compiling MazeBot...' -ForegroundColor Cyan
    & $CLI compile --fqbn $FQBN --libraries $Libraries --build-path $BuildPath $Sketch
    if ($LASTEXITCODE -ne 0) {
        throw 'Compile failed.'
    }

    Write-Host ''
    Write-Host '[2/2] Uploading via ST-LINK SWD...' -ForegroundColor Cyan
    & $CLI upload --fqbn $FQBN --input-dir $BuildPath $Sketch
    if ($LASTEXITCODE -ne 0) {
        throw 'Upload failed.'
    }

    Write-Host ''
    Write-Host 'Flash complete.' -ForegroundColor Green
    Wait-AndExit 0
}
catch {
    Write-Host ''
    Write-Host 'Flash failed.' -ForegroundColor Red
    Write-Host $_.Exception.Message -ForegroundColor Red
    Wait-AndExit 1
}
