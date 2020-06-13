__package__ = 'enums'
class FLIGHT_MODE(object):   
    MANUAL     = 0;
    STABLIZE     = 0;
    ACRO = 1;
    ALT_HOLD = 2;
    AUTO = 3;
    GUIDED = 4;
    LOITER = 5;
    RTL = 6;
    CIRCLE = 7;
    LAND = 9;

    MODEs= {
    '':0,
	0:'STABILIZE',
	2: 'ALT_HOLD',
	3: 'AUTO',
	4: 'GUIDED',
	5: 'LOITER',
	6: 'RTL',
	9: 'LAND',
	'STABILIZE':0,
	'ALT_HOLD':2,
	'AUTO':3,
	'GUIDED':4,
	'LOITER':5,
	'RTL':6,
	'LAND':9



	}
	# MODEs= {
	# '0':'STABILIZE',
	# '2': 'ALT_HOLD',
	# '3': 'AUTO',
	# '4': 'GUIDED',
	# '5': 'LOITER'
	# }