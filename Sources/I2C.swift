/*
 SwiftyGPIO


 Copyright (c) 2021 Craig Altenburg

 Based on code:
 Copyright (c) 2017 Umberto Raimondi
 
 Licensed under the MIT license, as follows:

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.)
 */
#if os(Linux)
    import Glibc
#else
    import Darwin.C
#endif

import Foundation
import SwiftyGPIO

// ============================================================================
//  Extension SwiftyGPIO
// ============================================================================
/// Allow the selection of which I2C ports are available based on platform.

extension SwiftyGPIO 
{
    // ------------------------------------------------------------------------
    //  Function hardwareI2Cs
    // ------------------------------------------------------------------------
    /// Return a dictionary of I2C instances indexed by I2C port number

    public static func hardwareI2Cs( for board: SupportedBoard ) -> [Int:I2CInterface]
    {
        switch board 
        {
        case .RaspberryPiRev1,
             .RaspberryPiRev2,
             .RaspberryPiPlusZero,
             .RaspberryPiZero2,
             .RaspberryPi2,
             .RaspberryPi3,
             .RaspberryPi4:
            return  [
                        0: I2C( i2cId: 0 ),
                        1: I2C( i2cId: 1 ),
                    ]

        case .CHIP:
            // i2c.0: connected to the AXP209 chip so is unavailable to this class
            // i2c.1: after 4.4.13-ntc-mlc connected to the U13 header I2C interface
            // i2c.2: connected to the U14 header I2C interface, XIO gpios are connected on this bus

            return  [
                        1: I2C( i2cId: 1 ),
                        2: I2C( i2cId: 2 )
                    ]

        default:
            return [:]
        }
    }
}

// ============================================================================
//  Class I2CInterface
// ============================================================================

public protocol I2CInterface 
{
    func isReachable  ( _ slaveAddress: Int ) -> Bool
    func setPEC       ( _ slaveAddress: Int, enabled: Bool) throws

    func readByte     ( _ slaveAddress: Int ) throws -> UInt8
    func readByte     ( _ slaveAddress: Int, command: UInt8) throws -> UInt8
    func readWord     ( _ slaveAddress: Int, command: UInt8) throws -> UInt16
    func readData     ( _ slaveAddress: Int, command: UInt8) throws -> [UInt8]
    func readI2CData  ( _ slaveAddress: Int, command: UInt8) throws -> [UInt8]
    func readRaw      ( _ slaveAddress: Int, length: Int) throws -> [UInt8]

    func writeQuick   ( _ slaveAddress: Int ) throws
    func writeByte    ( _ slaveAddress: Int, value: UInt8) throws
    func writeByte    ( _ slaveAddress: Int, command: UInt8, value: UInt8) throws
    func writeWord    ( _ slaveAddress: Int, command: UInt8, value: UInt16) throws
    func writeData    ( _ slaveAddress: Int, command: UInt8, values: [UInt8]) throws
    func writeI2CData ( _ slaveAddress: Int, command: UInt8, values: [UInt8]) throws
    
    func writeAndRead ( _ slaveAddress: Int, write: [UInt8], readLength: UInt) throws -> [UInt8]
}

// ============================================================================
//  Class I2C
// ============================================================================
/// Hardware I2C(SMBus) via SysFS using I2C_SMBUS ioctl

public final class I2C: I2CInterface 
{
    let i2cId: Int
    var fd: Int32 = -1
    var currentSlave: Int = -1

    // ------------------------------------------------------------------------
    //  Constructor
    // ------------------------------------------------------------------------

    public init( i2cId: Int ) 
    {
        self.i2cId=i2cId
    }

    // ------------------------------------------------------------------------
    //  Destructor
    // ------------------------------------------------------------------------
    
    deinit 
    {
        if fd != -1 
        {
            closeI2C()
        }
    }

    // ------------------------------------------------------------------------
    //  Function readByte (without command)
    // ------------------------------------------------------------------------

    public func readByte( _ slaveAddress: Int ) throws -> UInt8 
    {
        try setSlaveAddress( slaveAddress )

        var data : [UInt8] = [ 0 ]

        let r = smbus_ioctl(rw:      I2C_SMBUS_READ,
                            command: 0,
                            size:    I2C_SMBUS_BYTE,
                            data:    &data )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

        return data[0]
    }

    // ------------------------------------------------------------------------
    //  Function readByte (with command)
    // ------------------------------------------------------------------------

    public func readByte( _ slaveAddress: Int, command: UInt8 ) throws -> UInt8 
    {
        try setSlaveAddress( slaveAddress )

        var data : [UInt8] = [ 0 ]

        let r = smbus_ioctl(rw:      I2C_SMBUS_READ,
                            command: command,
                            size:    I2C_SMBUS_BYTE_DATA,
                            data:    &data)

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

        return data[0];
    }

    // ------------------------------------------------------------------------
    //  Function readWord (with command)
    // ------------------------------------------------------------------------
    /// Read a 16-bit little-endian word

    public func readWord( _ slaveAddress: Int, command: UInt8 ) throws -> UInt16 
    {
        try setSlaveAddress( slaveAddress )

        var data = [UInt8]( repeating: 0, count: 2 )

        let r = smbus_ioctl(rw: I2C_SMBUS_READ,
                            command: command,
                            size: I2C_SMBUS_WORD_DATA,
                            data: &data)

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

        return (UInt16( data[1] ) << 8) + UInt16( data[0] )
    }

    // ------------------------------------------------------------------------
    //  Function readData (with command)
    // ------------------------------------------------------------------------
    /// Read data from SMB.
    ///
    /// The first byte read indicates the length of the data.

    public func readData( _ slaveAddress: Int, command: UInt8 ) throws -> [UInt8] 
    {
        try setSlaveAddress( slaveAddress )

        var data = [UInt8]( repeating:0, count: I2C_DEFAULT_PAYLOAD_LENGTH )

        let r = smbus_ioctl( rw:      I2C_SMBUS_READ,
                             command: command,
                             size:    I2C_SMBUS_BLOCK_DATA,
                             data:    &data )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

        guard data[0] <=  I2C_DEFAULT_PAYLOAD_LENGTH else { throw POSIXError( POSIXError.EMSGSIZE ) }

        return Array<UInt8>( data[ 1 ... Int( data[0] )] )
    }

    // ------------------------------------------------------------------------
    //  Function readI2CData (with command)
    // ------------------------------------------------------------------------

    public func readI2CData( _ slaveAddress: Int, command: UInt8 ) throws -> [UInt8] 
    {
        try setSlaveAddress( slaveAddress )

        var data = [UInt8]( repeating: 0, count: I2C_DEFAULT_PAYLOAD_LENGTH )

        let r = smbus_ioctl( rw:      I2C_SMBUS_READ,
                             command: command,
                             size:    I2C_SMBUS_I2C_BLOCK_DATA,
                             data:    &data )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

        guard r <= I2C_DEFAULT_PAYLOAD_LENGTH else { throw POSIXError( POSIXError.EMSGSIZE ) }

        return Array<UInt8>( data[0 ..< Int( r )] )
    }

    // ------------------------------------------------------------------------
    //  Function readRaw (without command)
    // ------------------------------------------------------------------------

    public func readRaw(_ slaveAddress: Int, length: Int) throws -> [UInt8] 
    {
        var buf: [UInt8] = [UInt8](repeating:0, count: Int( length ))

        try setSlaveAddress( slaveAddress )

        let r =  read( fd, &buf, length )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

        return buf
    }

    // ------------------------------------------------------------------------
    //  Function writeAndRead  (without command)
    // ------------------------------------------------------------------------

    public func writeAndRead(_ slaveAddress: Int, write: [UInt8], readLength: UInt) throws -> [UInt8]  
    {
        guard    write.count<=I2C_DEFAULT_PAYLOAD_LENGTH 
              && readLength<=I2C_DEFAULT_PAYLOAD_LENGTH else { throw POSIXError( POSIXError.EINVAL ) }

        try setSlaveAddress( slaveAddress )

        var writeData = write

        var readBuf: [UInt8] = [UInt8](repeating:0, count: Int( readLength ))

        try writeData.withUnsafeMutableBufferPointer 
        { writePtr in

            try readBuf.withUnsafeMutableBufferPointer 
            { readPtr in

                let msgs = [ i2c_rw_msg( addr: UInt16( currentSlave ),
                                         flags: 0,
                                         len:   UInt16( writePtr.count ),
                                         buf:   writePtr.baseAddress! ),
                             i2c_rw_msg( addr:  UInt16( currentSlave ),
                                         flags: I2C_M_RD,
                                         len:   UInt16( readPtr.count ),
                                         buf:   readPtr.baseAddress! )
                           ]

                try msgs.withUnsafeBufferPointer 
                { msgsPtr in

                    var cmd = i2c_rdwr_ioctl_data( msgs: msgsPtr.baseAddress!, nmsgs: 2 )

                    let r = ioctl( fd, I2C_RDWR, &cmd )
                    
                    guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
                }
            }
        }      

        return readBuf              
    }

    // ------------------------------------------------------------------------
    //  Function writeQuick  (without command)
    // ------------------------------------------------------------------------
    /// Write quick
    ///
    /// This just sends a single byte over the I2C interface that contains the
    /// slave address with the R/W bit set to 0 (write).  It transfers no data
    /// to the slave.

    public func writeQuick(_ slaveAddress: Int) throws
    {
        try setSlaveAddress( slaveAddress )

        let r = smbus_ioctl( rw:      0,
                             command: 0,
                             size:    I2C_SMBUS_QUICK,
                             data:    nil )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
    }

    // ------------------------------------------------------------------------
    //  Function writeByte  (without command)
    // ------------------------------------------------------------------------

    public func writeByte( _ slaveAddress: Int, value: UInt8 ) throws
    {
        try setSlaveAddress( slaveAddress )

        let r = smbus_ioctl( rw:      I2C_SMBUS_WRITE,
                             command: value,
                             size:    I2C_SMBUS_BYTE,
                             data:    nil )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
    }

    // ------------------------------------------------------------------------
    //  Function writeByte  (with command)
    // ------------------------------------------------------------------------

    public func writeByte( _ slaveAddress: Int, command: UInt8, value: UInt8 ) throws
    {
        try setSlaveAddress( slaveAddress )

        var data : [UInt8] = [value]

        let r = smbus_ioctl( rw:      I2C_SMBUS_WRITE,
                             command: command,
                             size:    I2C_SMBUS_BYTE_DATA,
                             data:    &data )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
    }

    // ------------------------------------------------------------------------
    //  Function writeByte  (with command)
    // ------------------------------------------------------------------------

    public func writeWord( _ slaveAddress: Int, command: UInt8, value: UInt16 ) throws
    {
        try setSlaveAddress( slaveAddress )

        var data : [UInt8] = [UInt8( value & 0x00FF ), UInt8 (value >> 8 )]

        let r = smbus_ioctl( rw:      I2C_SMBUS_WRITE,
                             command: command,
                             size:    I2C_SMBUS_WORD_DATA,
                             data:    &data )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
    }

    // ------------------------------------------------------------------------
    //  Function writeData (with command)
    // ------------------------------------------------------------------------

    public func writeData(_ slaveAddress: Int, command: UInt8, values: [UInt8]) throws
    {
        guard values.count <= I2C_DEFAULT_PAYLOAD_LENGTH else { throw POSIXError( POSIXError.EINVAL ) }

        try setSlaveAddress( slaveAddress )

        var data = [UInt8]( repeating:0, count: values.count + 1 )

        for i in 1...values.count {
            data[i] = values[i-1]
        }

        data[0] = UInt8( values.count )

        let r = smbus_ioctl( rw:      I2C_SMBUS_WRITE,
                             command: command,
                             size:    I2C_SMBUS_BLOCK_DATA,
                             data:    &data )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
    }

    // ------------------------------------------------------------------------
    //  Function writeI2CData (with command)
    // ------------------------------------------------------------------------

    public func writeI2CData( _ slaveAddress: Int, command: UInt8, values: [UInt8] ) throws
    {
        try setSlaveAddress( slaveAddress )

        var valuesData = values

        let r = smbus_ioctl(rw:      I2C_SMBUS_WRITE,
                            command: command,
                            size:    Int32( values.count ),
                            data:    &valuesData )


        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
    }
 
    // ------------------------------------------------------------------------
    //  Function isReachable
    // ------------------------------------------------------------------------
    /// Check to see if an I2C slave is reachable.
    ///
    /// Mimic the behavior  of i2cdetect, performing either a readByte or 
    /// writeQuick depending on the slave;s address.
    ///
    ///     @param slaveAddress - the address of the slave to communicate with.
    ///
    ///     @returns `true` if the I2C request succeeded or `false` on an error.

    public func isReachable( _ slaveAddress: Int ) -> Bool 
    {

        do 
        {
            switch( slaveAddress )
            {
            case 0x3...0x2f, 0x38...0x4f, 0x60...0x77:
                try writeQuick( slaveAddress )
            default:
                _ = try readByte( slaveAddress )
            }
        } 
        catch
        {
            return false
        }

        return true
    }

    // ------------------------------------------------------------------------
    //  Function setPEC
    // ------------------------------------------------------------------------

    public func setPEC( _ slaveAddress: Int, enabled: Bool ) throws 
    {
        try setSlaveAddress( slaveAddress )

        let r =  ioctl( fd, I2C_PEC, enabled ? 1 : 0 )

        guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }
    }
 
    // ------------------------------------------------------------------------
    //  Private Function setSlaveAddress
    // ------------------------------------------------------------------------

    private func setSlaveAddress( _ slaveAddress: Int ) throws 
    {
        if fd == -1 
        {
            try openI2C()
        }

        if slaveAddress != currentSlave 
        {
            let r = ioctl( fd, I2C_SLAVE_FORCE, CInt( slaveAddress )) 
        
            guard r >= 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

            currentSlave = slaveAddress
        }
    }
 
    // ------------------------------------------------------------------------
    //  Private Function setSlaveAddress
    // ------------------------------------------------------------------------

    private func openI2C() throws 
    {
        let fd = open( I2CBASEPATH + String( i2cId ), O_RDWR )

        guard fd > 0 else { throw POSIXError( POSIXError.Code( rawValue: errno ) ?? POSIXError.EIO ) }

        self.fd = fd
    }
 
    // ------------------------------------------------------------------------
    //  Private Function setSlaveAddress
    // ------------------------------------------------------------------------

    private func closeI2C() 
    {
        close(fd)
    }

    // ------------------------------------------------------------------------
    //  Private Function smbus_ioctl
    // ------------------------------------------------------------------------
    
    private func smbus_ioctl( rw: UInt8, command: UInt8, size: Int32, data: UnsafeMutablePointer<UInt8>? ) -> Int32 
    {
        var args = i2c_smbus_ioctl_data( read_write: rw,
                                         command:    command,
                                         size:       size,
                                         data:       data )
        
        return ioctl( fd, I2C_SMBUS, &args )
    }

    // ------------------------------------------------------------------------
    //  Private Structures
    // ------------------------------------------------------------------------
    
    private struct i2c_smbus_ioctl_data 
    {
        var read_write: UInt8
        var command:    UInt8
        var size:       Int32
        var data:       UnsafeMutablePointer<UInt8>? //union: UInt8, UInt16, [UInt8]33
    }

    private struct i2c_rw_msg 
    {
        var addr:  UInt16
        var flags: UInt16
        var len:   UInt16
        var buf:   UnsafeMutablePointer<UInt8>
    }

    private struct i2c_rdwr_ioctl_data
    {
        var msgs:  UnsafePointer<i2c_rw_msg>
        var nmsgs: UInt32
    };
}

// MARK: - I2C/SMBUS Constants
internal let I2C_SMBUS_READ: UInt8 =   1
internal let I2C_SMBUS_WRITE: UInt8 =  0

internal let I2C_SMBUS_QUICK: Int32 = 0
internal let I2C_SMBUS_BYTE: Int32 = 1
internal let I2C_SMBUS_BYTE_DATA: Int32 = 2
internal let I2C_SMBUS_WORD_DATA: Int32 = 3
internal let I2C_SMBUS_BLOCK_DATA: Int32 = 5
//Not implemented: I2C_SMBUS_I2C_BLOCK_BROKEN  6
//Not implemented:  I2C_SMBUS_BLOCK_PROC_CALL   7
internal let I2C_SMBUS_I2C_BLOCK_DATA: Int32 = 8

internal let I2C_SLAVE: UInt = 0x703
internal let I2C_SLAVE_FORCE: UInt = 0x706
internal let I2C_RDWR: UInt = 0x707
internal let I2C_PEC: UInt = 0x708
internal let I2C_SMBUS: UInt = 0x720
internal let I2C_DEFAULT_PAYLOAD_LENGTH: Int = 32
internal let I2CBASEPATH="/dev/i2c-"

internal let I2C_M_RD: UInt16 = 0x0001

// MARK: - Darwin / Xcode Support
#if os(OSX) || os(iOS)
    private var O_SYNC: CInt { fatalError("Linux only") }
#endif
