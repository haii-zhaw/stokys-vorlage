/**
 * Register
 */
const REG_SERVO_TARGET_POSITION =   0x10;
const REG_SERVO_SETTLING_TIME =     0x14;
const REG_SERVO_PWM_CONFIGURATION = 0x18;
const SUB_SERVO_PWM_PERIOD =        0x01;
const SUB_SERVO_PWM_PULSE_MIN =     0x02;
const SUB_SERVO_PWM_PULSE_MAX =     0x03;
const REG_MOTOR_TARGET_POSITION =   0x20;
const REG_MOTOR_TARGET_VELOCITY =   0x24;
const REG_MOTOR_TARGET_VOLTAGE =    0x38;
const REG_MOTOR_ACTUAL_POSITION =   0x28;
const REG_MOTOR_ACTUAL_VELOCITY =   0x2C;
const REG_MOTOR_ACTRUAL_VOLTAGE =   0x3C;
const REG_MOTOR_MAX_VOLTAGE =       0x30;
const REG_MOTOR_CONTROL_MODE =      0x34;
const REG_SUPPLY_VOLTAGE =          0x60;
const REG_ANALOGIN_VOLTAGE =        0x61;
const REG_CONTROLLER_INFO =         0xFF;
const SUB_FIRMWARE_VERSION =        0x01;
const SUB_HARDWARE_VERSION =        0x02;

/**
 * Enums
 */
enum ServoMotors {
    Servo1 = 0,
    Servo2 = 1,
    Servo3 = 2,
    Servo4 = 3,
}

enum DCMotors {
    Motor1 = 0,
    Motor2 = 1,
    Motor3 = 2,
}

enum RobotMotors {
    MotorLinks = 2,
    MotorRechts = 0,
}

enum MotorControlMode {
    Position = 0,
    Velocity = 1,
    Voltage = 2,
}

enum AnalogInput {
    AnalogIn1 = 0,
    AnalogIn2 = 1,
}

enum DistanceSensor {
    Sensor1 = 1,
    Sensor2 = 0,
}

/**
 * Parameter
 */
const CONTROLLER_I2C_ADDRESS = 0x42;

/**
 * Stokys JS library compatibility
 */
const FIRMWARE_VERSION_COMPATIBILITY = "2.0.x" // Patch version does not matter


/**
* Custom Blocks
*/
//% weight=201 color=#fc0303 icon="\uf085"
//% groups=['Info', 'Kommunikation', 'Motoren', 'Servos', 'Inputs/Outputs']
namespace stokys {

    // #########################################################################
    // #### Analoge Inputs lesen ###############################################
    // #########################################################################

    //% blockId=analogInputBlock
    //% weight=99
    //% block="Spannung an %id"
    //% group="Inputs/Outputs"
    //% inlineInputMode=inline
    export function analogInputBlock(id: AnalogInput): number {
        let voltage = i2cRead(REG_ANALOGIN_VOLTAGE + id, 0x00, NumberFormat.UInt16LE);
        voltage = voltage / 1000.0;
        return voltage;
    }


    // #########################################################################
    // #### Distanz Sensor #####################################################
    // #########################################################################

    //% blockId=distanceSensorBlock
    //% weight=98
    //% block="Distanz bei %id"
    //% group="Inputs/Outputs"
    //% inlineInputMode=inline
    export function distanceSensorBlock(id: DistanceSensor): number {
        let voltage = i2cRead(REG_ANALOGIN_VOLTAGE + id, 0x00, NumberFormat.UInt16LE);
        voltage = voltage / 1000.0;
        
        const sensorMaxVoltage = 3.1;
        const sensorMinVoltage = 0.2;

        let x = voltage;

        // Limit input range
        if (x > sensorMaxVoltage) {
            x = sensorMaxVoltage;
        }
        else if (x < sensorMinVoltage) {
            x = sensorMinVoltage;
        }
        
        // Evaluate polynomial fitted to the GP2Y0AF15X distance sensor
        let y = 0.0;
        y += x ** 0 * 0.618999;
        y += x ** 1 * -2.30135;
        y += x ** 2 * 4.133445;
        y += x ** 3 * -4.122716;
        y += x ** 4 * 2.404736;
        y += x ** 5 * -0.815516;
        y += x ** 6 * 0.148858;
        y += x ** 7 * -0.011302;

        // Limit output range to 20.. 200 mm
        if (y > 0.2) {
            y = 0.2;
        } 
        else if (y < 0.02) {
            y = 0.02;
        }

        return y * 1000.0;
    }


    // #########################################################################
    // #### Versorgungsspannung messen #########################################
    // #########################################################################

    //% blockId=voltageSupplyBlock
    //% weight=97
    //% block="Versorgungsspannung"
    //% group="Inputs/Outputs"
    //% inlineInputMode=inline
    export function voltageSupplyBlock(): number {
        let voltage = i2cRead(REG_SUPPLY_VOLTAGE, 0x00, NumberFormat.UInt16LE);
        voltage = voltage / 1000.0;
        return voltage;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // #########################################################################
    // #### Roboter Positionsdelta vorgeben ####################################
    // #########################################################################

    /**
     * Roboter Positionsdelta vorgeben
     * @param id Motor Nummer
     * @param delta Positions-Delta
     */
    //% blockId=robotPositionDeltaBlock
    //% weight=90
    //% block="drehe %id um %delta Umdrehungen"
    //% group="Roboter"
    //% inlineInputMode=inline
    export function robotPositionDeltaBlock(id: RobotMotors, delta: number): void {
        if (id == RobotMotors.MotorLinks) {
            motorPositionDeltaBlock(DCMotors.Motor1, -1.0 * delta)
        } else if (id == RobotMotors.MotorRechts) {
            motorPositionDeltaBlock(DCMotors.Motor3, delta)
        }
    }


    // #########################################################################
    // #### Roboter warte bis angehalten #######################################
    // #########################################################################

    //% blockId=robotAwaitStop
    //% weight=89
    //% block="warte bis Motoren links und rechts anhalten"
    //% group="Roboter"
    //% inlineInputMode=inline
    export function robotAwaitStop(): void {
        const threshold = 1; // rpm
        while (getRobotMaxSpeed() < threshold);
        while (getRobotMaxSpeed() > threshold);
        return
    }

    function getRobotMaxSpeed(): number {
        let speed_left = motorGetSpeedBlock(DCMotors.Motor1);
        let speed_right = motorGetSpeedBlock(DCMotors.Motor3);
        return Math.max(Math.abs(speed_left), Math.abs(speed_right))
    }


    // #########################################################################
    // #### Roboter Geschwindigkeit ############################################
    // #########################################################################

    /**
     * Roboter Geschwindigkeit
     * @param speed_left Geschwindigkeit links
     * @param speed_right Geschwindigkeit rechts
     */
    //% blockId=roboterSpeedBlock
    //% weight=88
    //% block="drehe Motoren links und rechts mit Geschwindigkeit %speed_left und %speed_right rpm"
    //% group="Roboter"
    //% inlineInputMode=inline
    export function roboterSpeedBlock(speed_left: number, speed_right: number): void {
        motorSpeedBlock(DCMotors.Motor1, speed_left)
        motorSpeedBlock(DCMotors.Motor3, -1.0 * speed_right)
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // #########################################################################
    // #### Motor Positionsdelta vorgeben ######################################
    // #########################################################################

    /**
     * Motor Positionsdelta vorgeben
     * @param id Motor Nummer
     * @param delta Positions-Delta
     */
    //% blockId=motorPositionDeltaBlock
    //% weight=80
    //% block="drehe %id um %delta Umdrehungen"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorPositionDeltaBlock(id: DCMotors, delta: number): void {
        let position = i2cRead(REG_MOTOR_ACTUAL_POSITION + id, 0x00, NumberFormat.Int32LE);
        position = position + delta * 2.0 * Math.PI * 1000.0 // Convert to mrad
        i2cWrite(REG_MOTOR_TARGET_POSITION + id, 0, position, NumberFormat.Int32LE);
        i2cWrite(REG_MOTOR_CONTROL_MODE + id, 0x00, MotorControlMode.Position, NumberFormat.UInt8LE);
    }


    // #########################################################################
    // #### Motor Position messen ##############################################
    // #########################################################################

    /**
     * Motor Position messen
     * @param id Motor Nummer
     * @param delta Positions-Delta
     */
    //% blockId=motorGetPositionBlock
    //% weight=79
    //% block="Position von %id"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorGetPositionBlock(id: DCMotors): number {
        let position = i2cRead(REG_MOTOR_ACTUAL_POSITION + id, 0x00, NumberFormat.Int32LE);
        position = position / (2.0 * Math.PI * 1000.0); // Convert mrad to rotations
        return position;
    }


    // #########################################################################
    // #### Motor Position vorgeben ############################################
    // #########################################################################

    /**
     * Motor Position vorgeben
     * @param id Motor Nummer
     * @param position Position
     */
    //% blockId=motorPositionBlock
    //% weight=78
    //% block="drehe %id auf Position %position"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorPositionBlock(id: DCMotors, position: number): void {
        position = position * 2.0 * Math.PI * 1000.0 // Convert to mrad
        i2cWrite(REG_MOTOR_TARGET_POSITION + id, 0, position, NumberFormat.Int32LE);
        i2cWrite(REG_MOTOR_CONTROL_MODE + id, 0x00, MotorControlMode.Position, NumberFormat.UInt8LE);
    }


    // #########################################################################
    // #### Motor Geschwindigkeit vorgeben #####################################
    // #########################################################################

    /**
     * Motor Geschwindigkeit vorgeben
     * @param id Motor Nummer
     * @param speed Geschwindigkeit
     */
    //% blockId=motorSpeedBlock
    //% weight=77
    //% block="drehe %id mit Geschwindigkeit %speed rpm"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorSpeedBlock(id: DCMotors, speed: number): void {
        speed = speed / 60.0 * 2.0 * Math.PI * 1000.0; // Convert from rpm to mrad/s
        i2cWrite(REG_MOTOR_TARGET_VELOCITY + id, 0, speed, NumberFormat.Int32LE);
        i2cWrite(REG_MOTOR_CONTROL_MODE + id, 0x00, MotorControlMode.Velocity, NumberFormat.UInt8LE);
    }


    // #########################################################################
    // #### Motor Geschwindigkeit messen #######################################
    // #########################################################################

    /**
     * Motor Geschwindigkeit messen
     * @param id Motor Nummer
     */
    //% blockId=motorGetSpeedBlock
    //% weight=76
    //% block="Geschwindigkeit von %id"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorGetSpeedBlock(id: DCMotors): number {
        let speed = i2cRead(REG_MOTOR_ACTUAL_VELOCITY + id, 0x00, NumberFormat.Int32LE);
        speed = speed * (60.0 / (2.0 * Math.PI * 1000.0)); // Convert from mrad/s to rpm
        return speed;
    }


    // #########################################################################
    // #### Motor Spannung vorgeben ############################################
    // #########################################################################

    /**
     * Motor Spannung vorgeben
     * @param id Motor Nummer
     * @param voltage Spannung
     */
    //% blockId=motorVoltageBlock
    //% weight=75
    //% block="setze Spannung an %id auf %voltage V"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorVoltageBlock(id: DCMotors, voltage: number): void {
        voltage = voltage * 1000.0; // Convert from V to mV
        i2cWrite(REG_MOTOR_TARGET_VOLTAGE + id, 0, voltage, NumberFormat.Int32LE);
        i2cWrite(REG_MOTOR_CONTROL_MODE + id, 0x00, MotorControlMode.Voltage, NumberFormat.UInt8LE);
    }


    // #########################################################################
    // #### Motor Maximalspannung konfigurieren ################################
    // #########################################################################

    /**
     * Maximale Motorspannung konfigurieren
     * @param id Motor Nummer
     * @param voltage Spannung
     */
    //% blockId=motorMaxVoltageBlock
    //% weight=74
    //% block="konfiguriere maximale Motorspannung von %id auf %voltage V"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorMaxVoltageBlock(id: DCMotors, voltage: number): void {
        voltage = Math.abs(voltage) * 1000.0; // Convert from V to mV
        i2cWrite(REG_MOTOR_MAX_VOLTAGE + id, 0, voltage, NumberFormat.UInt16LE);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // #########################################################################
    // #### Servo Position vorgeben ############################################
    // #########################################################################

    /**
     * Servo Position vorgeben
     * @param id Servo Nummer
     * @param position Sollposition
     */
    //% blockId=writeServoBlock
    //% weight=70
    //% block="stelle Winkel %id %position grad"
    //% group="Servos"
    //% inlineInputMode=inline
    //% position.min=0 position.max=180
    export function writeServoBlock(id: ServoMotors, position: number): void {
        position = position * 1000; // convert to mdeg
        i2cWrite(REG_SERVO_TARGET_POSITION + id, 0, position, NumberFormat.UInt32LE);
    }


    // #########################################################################
    // #### Servo Geschwindigkeit konfigurieren ################################
    // #########################################################################

    /**
     * Servo Geschwindigkeit konfigurieren
     * @param id Servo Nummer
     * @param time Nachstellzeit
     */
    //% blockId=servoSpeedBlock
    //% weight=69
    //% block="setze Geschwindigkeit von %id auf %time s"
    //% group="Servos"
    //% inlineInputMode=inline
    //% position.min=0 position.max=10
    export function servoSpeedBlock(id: ServoMotors, time: number): void {
        time = time * 1000; // convert to ms
        i2cWrite(REG_SERVO_SETTLING_TIME + id, 0, time, NumberFormat.UInt32LE);
    }


    // #########################################################################
    // #### Servo PWM Konfiguration ############################################
    // #########################################################################

    /**
     * Servo PWM Konfiguration
     * @param id Servo Nummer
     * @param period Periode (typisch 20'000 us) in us
     * @param pulse_min Minimale puls-zeit (typisch 1000 us) in us
     * @param pulse_max Maximale puls-zeit (typisch 2000 us) in us
     */
    //% blockId=servoPWMBlock
    //% weight=68
    //% block="konfiguriere %id PWM mit Periode %periode s und Pulsdauer von %pulse_min bis %pulse_max s"
    //% group="Servos"
    //% inlineInputMode=inline
    export function servoPWMBlock(id: ServoMotors, period: number, pulse_min: number, pulse_max: number): void {
        period = period * 1e6
        pulse_min = pulse_min*1e6
        pulse_max = pulse_max*1e6
        i2cWrite(REG_SERVO_PWM_CONFIGURATION + id, SUB_SERVO_PWM_PERIOD, 0, NumberFormat.UInt16LE); // Suppress update (invalid period)
        i2cWrite(REG_SERVO_PWM_CONFIGURATION + id, SUB_SERVO_PWM_PULSE_MIN, pulse_min, NumberFormat.UInt16LE);
        i2cWrite(REG_SERVO_PWM_CONFIGURATION + id, SUB_SERVO_PWM_PULSE_MAX, pulse_max, NumberFormat.UInt16LE);
        i2cWrite(REG_SERVO_PWM_CONFIGURATION + id, SUB_SERVO_PWM_PERIOD, period, NumberFormat.UInt16LE);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // #########################################################################
    // #### Schriebe Stokys Controller Register ################################
    // #########################################################################

    /**
     * Schriebe Stokys Controller Register
     * @param index Index
     * @param subindex Subindex
     * @param value Wert
     * @param format Datentyp
     */
    //% blockId=i2cWrite
    //% weight=60
    //% block="schreibe Wert %value mit Datentyp %format nach %index:%subindex"
    //% group="Kommunikation"
    //% inlineInputMode=inline
    //% index.min=0 index.max=255
    //% subindex.min=0 index.max=255
    export function i2cWriteBlock(value: number, format: NumberFormat, index: number, subindex: number): void {
        i2cWrite(index, subindex, value, format);
    }


    // #########################################################################
    // #### Lese Stokys Controller Register ##############################################
    // #########################################################################

    /**
     * Lese Stokys Controller Register
     * @param index Index
     * @param subindex Subindex
     * @param format Datentyp
     * @returns Nummer
     */
    //% blockId=i2cRead
    //% weight=59
    //% block="Wert aus %index:%subindex mit Dateityp %format"
    //% group="Kommunikation"
    //% inlineInputMode=inline
    //% index.min=0 index.max=255
    //% subindex.min=0 index.max=255
    export function i2cReadBlock(index: number, subindex: number, format: NumberFormat): number {
        return i2cRead(index, subindex, format)
    }


    // #########################################################################
    // #### Schreibe Wert über I2C (intern) ####################################
    // #########################################################################

    function i2cWrite(index: number, subindex: number, value: number, format: NumberFormat): void {
        let payload: number[] = [index, subindex];
        let addressBuffer = Buffer.fromArray(payload);

        switch (format) {
            case NumberFormat.UInt32LE: { break };
            case NumberFormat.Int32LE: { break };
            case NumberFormat.UInt16LE: { break };
            case NumberFormat.Int16LE: { break };
            case NumberFormat.UInt8LE: { break };
            default: {
                control.fail("Unsupported value type");
            }
        }

        pins.i2cWriteBuffer(CONTROLLER_I2C_ADDRESS, addressBuffer, true);
        pins.i2cWriteNumber(CONTROLLER_I2C_ADDRESS, value, format);

        // let readback = i2cRead(index, subindex, format);
        // if (readback != value) {
        //     control.fail("I2C write error");
        // }
    }


    // #########################################################################
    // #### Lese Wert über I2C (intern) ########################################
    // #########################################################################

    function i2cRead(index: number, subindex: number, format: NumberFormat): number {
        let payload: number[] = [index, subindex];
        let addressBuffer = Buffer.fromArray(payload);

        switch (format) {
            case NumberFormat.UInt32LE: { break };
            case NumberFormat.Int32LE: { break };
            case NumberFormat.UInt16LE: { break };
            case NumberFormat.Int16LE: { break };
            case NumberFormat.UInt8LE: { break };
            default: {
                control.fail("Unsupported value type");
            }
        }

        pins.i2cWriteBuffer(0x42, addressBuffer, true);
        let num = pins.i2cReadNumber(0x42, format);

        return num;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // #########################################################################
    // #### Firmware Version auslesen ##########################################
    // #########################################################################

    /**
     * Firmware Version auslesen
     * @returns Version String
     */
    //% blockId=firmwareVersionBlock
    //% weight=50
    //% block="Firmware Version des Controllers"
    //% group="Info"
    //% inlineInputMode=inline
    export function firmwareVersionBlock(): string {
        let version = i2cRead(REG_CONTROLLER_INFO, SUB_FIRMWARE_VERSION, NumberFormat.UInt32LE);

        // Extract the major, minor, and patch versions from the uint32
        let major = version & 0xFF;
        let minor = (version >>> 8) & 0xFF;
        let patch = (version >>> 16) & 0xFF;
       
        // Convert the versions to strings
        let majorStr = major.toString();
        let minorStr = minor.toString();
        let patchStr = patch.toString();
       
        // Construct the version string
        let versionStr = `${majorStr}.${minorStr}.${patchStr}`;
       
        return versionStr;
    }   


    // #########################################################################
    // #### Hardware Version auslesen ##########################################
    // #########################################################################

    /**
     * Hardware Version auslesen
     * @returns Version String
     */
    //% blockId=hardwareVersionBlock
    //% weight=49
    //% block="Hardware Version des Controllers"
    //% group="Info"
    //% inlineInputMode=inline
    export function hardwareVersionBlock(): string {
        let version = i2cRead(REG_CONTROLLER_INFO, SUB_HARDWARE_VERSION, NumberFormat.UInt32LE);

        // Extract the major, minor, and patch versions from the uint32
        let major = version & 0xFF;
        let minor = (version >>> 8) & 0xFF;
        let patch = (version >>> 16) & 0xFF;
       
        // Convert the versions to strings
        let majorStr = major.toString();
        let minorStr = minor.toString();
        let patchStr = patch.toString();
       
        // Construct the version string
        let versionStr = `${majorStr}.${minorStr}.${patchStr}`;
       
        return versionStr;
    } 


    // #########################################################################
    // #### Firmware Kompatibilität ############################################
    // #########################################################################

    /**
     * Firmware Kompatibilität prüfen
     * @returns Bool
     */
    //% blockId=firmwareCompatibilityBlock
    //% weight=48
    //% block="Firmware-Kompatibilität"
    //% group="Info"
    //% inlineInputMode=inline
    export function firmwareCompatibilityBlock(): boolean {
        let v = firmwareVersionBlock();
        let compat = FIRMWARE_VERSION_COMPATIBILITY;

        let vs = v.split(".");
        let compats = compat.split(".");

        if (vs[0] != compats[0]) {
            // Major Version must match
            return false;
        }

        for (let i = 2; i >= 0 ; i--) {
            console.log(i)
            if (compats[i] == "x") {
                // No check if compat == "x"
                continue;
            }
            if (vs[i] < compats[i]) {
                // Controller firmware version must be same or higher as compat
                return false;
            }
        }

        return true;

    } 


    // #########################################################################
    // #### Firmware Kompatibilität prüfen #####################################
    // #########################################################################

    /**
     * Firmware Kompatibilität prüfen
     */
    //% blockId=firmwareCheckBlock
    //% weight=47
    //% block="prüfe Firmware-Kompatibilität"
    //% group="Info"
    //% inlineInputMode=inline
    export function firmwareCheckBlock(): void {
        if (firmwareCompatibilityBlock() == true) {
            basic.showIcon(IconNames.Heart)
        } else {
            basic.showIcon(IconNames.Asleep)
        }

        basic.showString(`  installiert: v${firmwareVersionBlock()}`)
        basic.showString(`  minimal: v${FIRMWARE_VERSION_COMPATIBILITY}`)

    } 
}


