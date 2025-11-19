Param(
  [string]$InputFile = ""
)
if (-not (Test-Path .\explore.exe)) {
  Write-Error "explore.exe not found. Run build.ps1 first."; exit 1
}
if ($InputFile -and (Test-Path $InputFile)) {
  Get-Content $InputFile | .\explore.exe
} else {
  .\explore.exe
}