param([string]$ProjectRoot = '')
$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $ProjectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
}
$taskRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
$taskSourcePath = Join-Path $taskRoot 'User\Log\fault_noinit.sct'
$taskDestinationPath = Join-Path $taskRoot 'MDK-ARM\rm_main.sct'
if (-not (Test-Path -LiteralPath $taskSourcePath -PathType Leaf)) {
    throw "缺少自定义 SCT：$taskSourcePath"
}
$taskSource = [IO.File]::ReadAllText($taskSourcePath)
foreach ($taskRegion in @('RW_FAULT_NOINIT', 'RW_LOG_SPI_DMA')) {
    if ($taskSource -notmatch ('\b' + $taskRegion + '\b')) {
        throw "自定义 SCT 缺少必要区域：$taskRegion"
    }
}

# 生成文件与源文件的目录不同，将直接包含的头文件转换为可解析的路径。
# 路径每次按当前工程计算；移动工程后重新编译即可更新生成的 SCT。
$taskSourceDir = Split-Path $taskSourcePath -Parent
$taskOutput = [regex]::Replace($taskSource, '(?m)^(\s*#include\s+)"([^"\r\n]+)"', [System.Text.RegularExpressions.MatchEvaluator]{
    param($taskMatch)
    $taskInclude = [IO.Path]::GetFullPath((Join-Path $taskSourceDir $taskMatch.Groups[2].Value))
    if (-not (Test-Path -LiteralPath $taskInclude -PathType Leaf)) {
        throw "SCT 包含的头文件不存在：$taskInclude"
    }
    return $taskMatch.Groups[1].Value + '"' + $taskInclude.Replace('\', '/') + '"'
})
if (-not (Test-Path -LiteralPath $taskDestinationPath) -or
    [IO.File]::ReadAllText($taskDestinationPath) -cne $taskOutput) {
    [IO.File]::WriteAllText($taskDestinationPath, $taskOutput, (New-Object Text.UTF8Encoding($false)))
}
Write-Output "Scatter synchronized: $taskDestinationPath"
