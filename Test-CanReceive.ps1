[CmdletBinding()]
param(
    [ValidateRange(0, 1)][int]$Channel = 0,
    [ValidateSet(3, 4)][int]$DeviceType = 4,
    [ValidateRange(1, 127)][int]$NodeId = 1,
    [string]$DllPath,
    [switch]$ListenOnly
)

$ErrorActionPreference = 'Stop'
if ([Environment]::Is64BitProcess) {
    throw 'Run this script with 32-bit Windows PowerShell (SysWOW64).'
}

if (!$DllPath) {
    $DllPath = Join-Path $PSScriptRoot '..\USB_CAN TOOL\ControlCAN.dll'
}
$dll = (Resolve-Path -LiteralPath $DllPath).Path
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;

public static class UsbCan
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Config
    {
        public uint AccCode, AccMask, Reserved;
        public byte Filter, Timing0, Timing1, Mode;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Frame
    {
        public uint Id, Timestamp;
        public byte TimeFlag, SendType, Remote, Extended, Length;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public byte[] Data;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public byte[] Reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Status
    {
        public byte Interrupt, Mode, Register, ArbitrationLost, ErrorCapture;
        public byte WarningLimit, RxErrors, TxErrors;
        public uint Reserved;
    }

    [DllImport(@"$dll", ExactSpelling = true)]
    public static extern uint VCI_OpenDevice(uint type, uint index, uint reserved);
    [DllImport(@"$dll", ExactSpelling = true)]
    public static extern uint VCI_CloseDevice(uint type, uint index);
    [DllImport(@"$dll", ExactSpelling = true)]
    public static extern uint VCI_InitCAN(uint type, uint index, uint channel, ref Config config);
    [DllImport(@"$dll", ExactSpelling = true)]
    public static extern uint VCI_StartCAN(uint type, uint index, uint channel);
    [DllImport(@"$dll", ExactSpelling = true)]
    public static extern uint VCI_Transmit(uint type, uint index, uint channel, ref Frame frame, uint count);
    [DllImport(@"$dll", ExactSpelling = true)]
    public static extern uint VCI_Receive(uint type, uint index, uint channel, [Out] Frame[] frames, uint count, int wait);
    [DllImport(@"$dll", ExactSpelling = true)]
    public static extern uint VCI_ReadCANStatus(uint type, uint index, uint channel, out Status status);
}
"@

if ([UsbCan]::VCI_OpenDevice($DeviceType, 0, 0) -ne 1) {
    throw "Cannot open USB-CAN device type $DeviceType, index 0."
}

try {
    $requestId = 0x600 + $NodeId
    $responseId = 0x580 + $NodeId
    $config = New-Object UsbCan+Config
    $config.AccMask = [uint32]::MaxValue
    $config.Filter = 1
    $config.Timing0 = 0x00
    $config.Timing1 = 0x1C
    if ($ListenOnly) { $config.Mode = 1 }
    if ([UsbCan]::VCI_InitCAN($DeviceType, 0, $Channel, [ref]$config) -ne 1) {
        throw "Cannot configure CAN channel $Channel."
    }
    if ([UsbCan]::VCI_StartCAN($DeviceType, 0, $Channel) -ne 1) {
        throw "Cannot start CAN channel $Channel."
    }
    Write-Host "USB-CAN channel $Channel started at 500 kbit/s."

    if (!$ListenOnly) {
        $request = New-Object UsbCan+Frame
        $request.Id = $requestId
        $request.Length = 8
        $request.SendType = 1
        $request.Data = [byte[]](0x40, 0x01, 0x20, 0, 0, 0, 0, 0)
        $request.Reserved = [byte[]]::new(3)
        $sent = [UsbCan]::VCI_Transmit($DeviceType, 0, $Channel, [ref]$request, 1)
        Write-Host ('TX {0:X3}  40 01 20 00 00 00 00 00  (adapter accepted: {1})' -f $requestId, $sent)
        if ($sent -ne 1) { throw 'Adapter did not accept the test frame.' }
    }

    $frames = New-Object 'UsbCan+Frame[]' 32
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $matched = $false
    while ($timer.ElapsedMilliseconds -lt 2500) {
        $count = [UsbCan]::VCI_Receive($DeviceType, 0, $Channel, $frames, $frames.Length, 50)
        if ($count -gt $frames.Length) { throw "USB-CAN receive failed ($count)." }
        for ($i = 0; $i -lt $count; $i++) {
            $frame = $frames[$i]
            $data = [BitConverter]::ToString($frame.Data, 0, [Math]::Min($frame.Length, 8)).Replace('-', ' ')
            Write-Host ('RX {0:X}  {1}' -f $frame.Id, $data)
            if ($frame.Id -eq $responseId -and !$frame.Extended -and !$frame.Remote -and
                $frame.Length -eq 8 -and $data.StartsWith('43 01 20 00 ')) {
                $matched = $true
            }
        }
        Start-Sleep -Milliseconds 10
    }
    $status = New-Object UsbCan+Status
    if ([UsbCan]::VCI_ReadCANStatus($DeviceType, 0, $Channel, [ref]$status) -eq 1) {
        Write-Host ('CAN status 0x{0:X2}, error capture 0x{1:X2}, RX errors {2}, TX errors {3}.' -f
            $status.Register, $status.ErrorCapture, $status.RxErrors, $status.TxErrors)
    }
    if (!$ListenOnly) {
        if (!$matched) {
            throw ('Expected STM32 SDO response {0:X3} / 43 01 20 00 xx xx xx xx was not received.' -f $responseId)
        }
        Write-Host 'PASS: matching SDO response received.'
    }
}
finally {
    [void][UsbCan]::VCI_CloseDevice($DeviceType, 0)
}
