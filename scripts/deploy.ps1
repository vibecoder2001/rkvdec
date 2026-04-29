# scripts/deploy.ps1 — copies a built driver package to a target RK3588
# and installs it via pnputil.
param(
    [Parameter(Mandatory=$true)] [string] $Target,
    [string[]] $Drivers = @("rkiommu", "rkmpp_ccu", "rkmpp")
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path "$PSScriptRoot\.."

foreach ($d in $Drivers) {
    $pkg = Get-ChildItem "$root\driver\$d\ARM64\Debug\$d" -ErrorAction SilentlyContinue
    if (-not $pkg) {
        Write-Warning "skipping $d — not built"
        continue
    }
    Write-Host "Deploying $d to $Target"
    $remote = "\\$Target\C`$\drvtest\$d"
    New-Item -ItemType Directory -Force -Path $remote | Out-Null
    Copy-Item "$($pkg.FullName)\*" $remote -Recurse -Force
    Invoke-Command -ComputerName $Target -ScriptBlock {
        param($d)
        pnputil /add-driver "C:\drvtest\$d\$d.inf" /install
    } -ArgumentList $d
}
