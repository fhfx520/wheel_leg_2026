param([string]$ProjectPath = '')
$ErrorActionPreference = 'Stop'

# 根据脚本所在位置定位工程，不依赖 CubeMX 的当前工作目录。
if ([string]::IsNullOrWhiteSpace($ProjectPath)) {
    $taskRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
    $ProjectPath = Join-Path $taskRoot 'MDK-ARM\rm_main.uvprojx'
}
$taskProject = (Resolve-Path -LiteralPath $ProjectPath).Path
$taskProjectDir = Split-Path $taskProject -Parent
$taskScatter = '..\User\Log\fault_noinit.sct'
$taskScatterPath = [IO.Path]::GetFullPath((Join-Path $taskProjectDir $taskScatter))
if (-not (Test-Path -LiteralPath $taskScatterPath -PathType Leaf)) {
    throw "自定义 SCT 不存在，未修改工程：$taskScatterPath"
}

$taskText = [IO.File]::ReadAllText($taskProject)
$taskXml = New-Object System.Xml.XmlDocument
$taskXml.LoadXml($taskText)
$taskNodes = $taskXml.SelectNodes('/Project/Targets/Target/TargetOption/TargetArmAds/LDads')
$taskBlocks = [regex]::Matches($taskText, '(?s)<LDads>.*?</LDads>')
if ($taskNodes.Count -eq 0 -or $taskNodes.Count -ne $taskBlocks.Count) {
    throw '不能唯一识别目标级链接设置，未修改工程。'
}

# 只修改链接器的两个字段，保留其他配置、缩进和换行。
$taskUpdated = [regex]::Replace($taskText, '(?s)<LDads>.*?</LDads>', [System.Text.RegularExpressions.MatchEvaluator]{
    param($taskMatch)
    $taskBlock = $taskMatch.Value
    foreach ($taskField in @('umfTarg', 'ScatterFile')) {
        $taskPattern = '(?s)<' + $taskField + '(?:\s*/>|>.*?</' + $taskField + '>)'
        if ([regex]::Matches($taskBlock, $taskPattern).Count -ne 1) {
            throw "链接器字段 $taskField 缺失或重复，未修改工程。"
        }
        $taskValue = if ($taskField -eq 'umfTarg') { '0' } else { $taskScatter }
        $taskReplacement = '<' + $taskField + '>' + $taskValue + '</' + $taskField + '>'
        $taskBlock = [regex]::Replace($taskBlock, $taskPattern, $taskReplacement)
    }
    return $taskBlock
})

# 先验证完整 XML，再备份并保存；重复执行不会反复改写。
$taskXml.LoadXml($taskUpdated)
if ($taskUpdated -ne $taskText) {
    Copy-Item -LiteralPath $taskProject -Destination ($taskProject + '.before_scatter_restore.bak') -Force
    [IO.File]::WriteAllText($taskProject, $taskUpdated, (New-Object Text.UTF8Encoding($false)))
    Write-Output "Restored scatter file: $taskScatter"
} else {
    Write-Output "Scatter file already correct: $taskScatter"
}
exit 0
