# Sets up the wired Ethernet port for the two-PC command-link test.
# Must run as administrator: use tools\network_setup.bat (it asks for permission).
#
#   -Role rover  -> 10.10.10.5   (runs rover-node)
#   -Role base   -> 10.10.10.4   (runs rover-base, gamepad plugged in here)
#
# It changes only the wired Ethernet adapter (Wi-Fi and internet are untouched):
#   * static IPv4 address, /24, no gateway, no DNS
#   * network profile = Private (a direct cable is otherwise "Public" and blocked)
#   * firewall: allow inbound UDP 5005-5006 (command link) and ping, Private profile only
#
# Undo: tools\network_setup.bat -Undo   (back to automatic/DHCP, removes the firewall rules)

param(
    [ValidateSet("rover", "base")] [string]$Role,
    [string]$Adapter,
    [switch]$Undo
)

$ErrorActionPreference = "Stop"
trap {
    Write-Host ""
    Write-Host "ERROR: $_" -ForegroundColor Red
    Read-Host "Press Enter to close"
    exit 1
}
$RuleName = "Rover command link (UDP 5005-5006)"
$PingRule = "Rover command link (ping)"

function Fail($msg) { Write-Host "STOPPED: $msg" -ForegroundColor Red; Read-Host "Press Enter to close"; exit 1 }

$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Fail "Needs administrator rights. Run tools\network_setup.bat instead."
}

# ---- pick the wired adapter
$wired = @(Get-NetAdapter -Physical | Where-Object { $_.MediaType -eq "802.3" })
if ($Adapter) { $wired = @($wired | Where-Object { $_.Name -eq $Adapter }) }
if ($wired.Count -eq 0) { Fail "No wired Ethernet adapter found. Plug in the cable / USB-Ethernet adapter and try again." }
if ($wired.Count -gt 1) {
    Write-Host "Several wired adapters found:"
    $i = 0
    foreach ($a in $wired) { Write-Host ("  [{0}] {1}  ({2}, {3})" -f $i, $a.Name, $a.InterfaceDescription, $a.Status); $i++ }
    $pick = Read-Host "Number of the one connected to the other PC"
    $wired = @($wired[[int]$pick])
}
$nic = $wired[0]
Write-Host ("Using adapter: {0} ({1}), status {2}" -f $nic.Name, $nic.InterfaceDescription, $nic.Status)

if ($Undo) {
    Set-NetIPInterface -InterfaceAlias $nic.Name -Dhcp Enabled
    Set-DnsClientServerAddress -InterfaceAlias $nic.Name -ResetServerAddresses
    Get-NetIPAddress -InterfaceAlias $nic.Name -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.IPAddress -like "10.10.10.*" } | Remove-NetIPAddress -Confirm:$false
    Get-NetFirewallRule -DisplayName $RuleName -ErrorAction SilentlyContinue | Remove-NetFirewallRule
    Get-NetFirewallRule -DisplayName $PingRule -ErrorAction SilentlyContinue | Remove-NetFirewallRule
    Write-Host "Undone: adapter back to automatic (DHCP), rover firewall rules removed." -ForegroundColor Green
    Read-Host "Press Enter to close"
    exit 0
}

if (-not $Role) {
    $answer = Read-Host "Is this PC the ROVER (r) or the BASE STATION with the gamepad (b)?"
    if ($answer -match "^r") { $Role = "rover" } elseif ($answer -match "^b") { $Role = "base" } else { Fail "Answer r or b." }
}
$ip = "10.10.10.5"
if ($Role -eq "base") { $ip = "10.10.10.4" }
$other = "10.10.10.4"
if ($Role -eq "base") { $other = "10.10.10.5" }

# ---- static IP
Set-NetIPInterface -InterfaceAlias $nic.Name -Dhcp Disabled
Get-NetIPAddress -InterfaceAlias $nic.Name -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
New-NetIPAddress -InterfaceAlias $nic.Name -IPAddress $ip -PrefixLength 24 | Out-Null
Write-Host "IP address set to $ip/24 (no gateway, so internet stays on Wi-Fi)" -ForegroundColor Green

# ---- Private profile (may take a few seconds to appear after the cable is plugged in)
$netProfile = $null
for ($t = 0; $t -lt 10 -and -not $netProfile; $t++) {
    $netProfile = Get-NetConnectionProfile -InterfaceAlias $nic.Name -ErrorAction SilentlyContinue
    if (-not $netProfile) { Start-Sleep -Seconds 1 }
}
if ($netProfile) {
    Set-NetConnectionProfile -InterfaceAlias $nic.Name -NetworkCategory Private
    Write-Host "Network profile set to Private" -ForegroundColor Green
} else {
    Write-Host "Could not set the network profile (is the cable plugged in at both ends?). Run this again once the link light is on." -ForegroundColor Yellow
}

# ---- firewall
Get-NetFirewallRule -DisplayName $RuleName -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName $RuleName -Direction Inbound -Protocol UDP -LocalPort 5005-5006 -Profile Private -Action Allow | Out-Null
Get-NetFirewallRule -DisplayName $PingRule -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName $PingRule -Direction Inbound -Protocol ICMPv4 -IcmpType 8 -Profile Private -Action Allow | Out-Null
Write-Host "Firewall: allowed UDP 5005-5006 and ping on Private networks" -ForegroundColor Green

Write-Host ""
Write-Host "This PC is the $($Role.ToUpper()) at $ip." -ForegroundColor Green
Write-Host "Once the other PC is set up too, test with:  ping $other"
Read-Host "Press Enter to close"
