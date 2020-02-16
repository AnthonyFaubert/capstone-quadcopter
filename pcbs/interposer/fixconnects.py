#!/usr/bin/python3

data = '''3V" pad="55 56"/>
5V" pad="53 54"/>
BOOT0" pad="71"/>
GND" pad="1 2 5 23 49 50 51 52 99 100"/>
NC" pad="48"/>
NRST" pad="6"/>
PA0" pad="12"/>
PA1" pad="11"/>
PA10" pad="91"/>
PA13" pad="92"/>
PA14" pad="89"/>
PA15" pad="90"/>
PA2" pad="14"/>
PA3" pad="13"/>
PA4" pad="16"/>
PA5" pad="15"/>
PA6" pad="18"/>
PA7" pad="17"/>
PA8" pad="93"/>
PA9" pad="94"/>
PB0" pad="22"/>
PB1" pad="21"/>
PB10" pad="34"/>
PB11" pad="35"/>
PB12" pad="36"/>
PB13" pad="37"/>
PB14" pad="38"/>
PB15" pad="39"/>
PB2" pad="24"/>
PB3" pad="78"/>
PB4" pad="75"/>
PB5" pad="76"/>
PB6" pad="73"/>
PB7" pad="74"/>
PB8" pad="69"/>
PB9" pad="70"/>
PC0" pad="8"/>
PC1" pad="7"/>
PC10" pad="87"/>
PC11" pad="88"/>
PC12" pad="85"/>
PC13" pad="62"/>
PC14" pad="59"/>
PC15" pad="60"/>
PC2" pad="10"/>
PC3" pad="9"/>
PC4" pad="20"/>
PC5" pad="19"/>
PC6" pad="97"/>
PC7" pad="98"/>
PC8" pad="95"/>
PC9" pad="96"/>
PD0" pad="86"/>
PD1" pad="83"/>
PD10" pad="42"/>
PD11" pad="43"/>
PD12" pad="44"/>
PD13" pad="45"/>
PD14" pad="46"/>
PD15" pad="47"/>
PD2" pad="84"/>
PD3" pad="81"/>
PD4" pad="82"/>
PD5" pad="79"/>
PD6" pad="80"/>
PD7" pad="77"/>
PD8" pad="40"/>
PD9" pad="41"/>
PE0" pad="67"/>
PE1" pad="68"/>
PE10" pad="28"/>
PE11" pad="29"/>
PE12" pad="30"/>
PE13" pad="31"/>
PE14" pad="32"/>
PE15" pad="33"/>
PE2" pad="65"/>
PE3" pad="66"/>
PE4" pad="63"/>
PE5" pad="64"/>
PE6" pad="61"/>
PE7" pad="25"/>
PE8" pad="26"/>
PE9" pad="27"/>
PH0" pad="57"/>
PH1" pad="58"/>
VDD" pad="3 4 72"/>'''


def pinToConnectTo(pin):
    if (1 <= pin) and (pin <= 50):
        if (pin%2 == 1):
            return 101 + int(pin/2)
        else:
            return 125 + int(pin/2)
    if (51 <= pin) and (pin <= 100):
        if (pin%2 == 1):
            return 151 + int((pin-50)/2)
        else:
            return 175 + int((pin-50)/2)
    raise Exception("Invalid pin input or there's a bug.")

def procLine(ln):
    fromp = ln[:ln.find('"')]
    toList = ln[ln.find('"')+7 : -3].split()
    for i in range(len(toList)):
        toList[i] = int(toList[i])
    return (fromp, tuple(toList))

def connect(fromp, to):
    newTo = []
    for pin in to:
        newTo.append(pin)
        newTo.append(pinToConnectTo(pin))
    return (fromp, tuple(newTo))

def conns2xml(fromp, to):
    return '<connect gate="G$1" pin="%s" pad="%s"/>' % (fromp, ' '.join(str(pin) for pin in to))

def test():
    test_ins = [1,3,5,49,2,4,6,50,51,53,55,99,52,54,56,100]
    test_outs = [101,102,103,125,126,127,128,150,151,152,153,175,176,177,178,200]
    for i in range(len(test_ins)):
        assert(pinToConnectTo(test_ins[i]) == test_outs[i])

    a = procLine('PH0" pad="57"/>')
    assert(a[0] == 'PH0' and len(a[1]) == 1 and a[1][0] == 57)
    a = procLine('VDD" pad="3 4 72"/>')
    assert(a[0] == 'VDD' and len(a[1]) == 3 and a[1][0] == 3 and a[1][1] == 4 and a[1][2] == 72)


test()
for line in data.split('\n'):
    newConns = connect(*procLine(line))
    print(conns2xml(*newConns))
    
