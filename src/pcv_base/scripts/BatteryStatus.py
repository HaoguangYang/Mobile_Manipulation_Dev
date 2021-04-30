import pymysql.cursors    #For SQL Database
import matplotlib.pyplot as plt

def BatteryStatus(NodeNo):
    
    #Connection to the Database
    db = pymysql.connect(
                host = "ldvinmac01.itap.purdue.edu",
                port = 3317,
                user = "Om-students",
                password = "hN$2aIb6L",
                database = "Voyles_robots"
                )
    
    dbName = "Voyles_robots"
    
    TelTableName = "CARTMAN"+NodeNo.zfill(2)+"_TELEMETRY"
    
    
    try:
        with db:
            with db.cursor() as cursor:
                sql = "SELECT * FROM `"+dbName+"`.`"+TelTableName+"` ORDER BY `TimeStamp` DESC"
                cursor.execute(sql)
              
                BatteryData = cursor.fetchone()
                TimeStampNow = BatteryData[0]
                VoltNow = BatteryData[1]
                TimeStampSel = TimeStampNow
                VoltSel = VoltNow
                TimeStamplst = [TimeStampNow]
                Voltlst = [VoltNow]
                Timedifflst = [0]
                
                BatteryData = cursor.fetchone()
                TimeStampPrev = BatteryData[0]
                VoltPrev = BatteryData[1]
                
                while VoltSel <= VoltPrev + 2:
                    TimeStamplst.append(TimeStampPrev)
                    Voltlst.append(VoltPrev)
                    Timediff = TimeStampSel - TimeStampPrev
                    Timedifflst.append(-Timediff.total_seconds()/60)
                    VoltSel = VoltPrev
                    BatteryData = cursor.fetchone()
                    TimeStampPrev = BatteryData[0]
                    VoltPrev = BatteryData[1]
                    
         
    finally:
        if db.open == True:
            db.close()
    
    
    
        
    dVlst = [0]
    
    for t in range(len(TimeStamplst)):
        if t==0:
            pass
        if -Timedifflst[t]<=10:
            tInd = t
        dV = (Voltlst[t]-Voltlst[t-1])/(Timedifflst[t]-Timedifflst[t-1])
        dVlst.append(dV)
        
    dVavg = 0    
    
    for d in range(tInd):
        if d==0:
            pass
        else:
            dVavg+=dVlst[d]/(len(dVlst)-1)
            
    PredVoltlst = []
    
    for t in Timedifflst:
        PredVoltlst.append(t*dVavg+VoltNow)
        
    
            
    plt.plot(Timedifflst, Voltlst, label = "Measured")
    plt.plot(Timedifflst, PredVoltlst, label = "Trend", linestyle = 'dotted')
    plt.suptitle('Battery Voltage Over Time')
    plt.grid()
    plt.xlabel('Time Passed (min)')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.show()
        
    print('TimeStamp             Volt (V)   dV/dt (mV/min)')
    
    for i in range(len(TimeStamplst)):
        print(f'{TimeStamplst[i]}   {Voltlst[i]:>8}       {round(1000*dVlst[i],3):>10}')
        
    EstTimeRem = -VoltNow/dVavg
    print(f'\nEstimated Time Remaining: {EstTimeRem//60:.0f} hour(s) {EstTimeRem%60:.3f} min(s)')

BatteryStatus('5')