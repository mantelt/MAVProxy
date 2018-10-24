#!/usr/bin/env python
'''
RSSIreport Module
Thomas Mantel, October 2018

inject GSM_LINK_STATUS msg based on LTE USB stick
'''

import serial
from pymavlink import mavutil
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

class RSSIreport(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(RSSIreport, self).__init__(mpstate, "RSSIreport", "")
        self.status_callcount = 0
        self.report_interval = 1 # seconds
        self.report_low_interval = 60 # seconds
        self.last_report = time.time()
        self.last_low = 0
        self.gsm_link_status_msg = mavutil.mavlink.MAVLink_gsm_link_status_message(
            0,
            mavutil.mavlink.GSM_MODEM_TYPE_HUAWEI_E3372,
            mavutil.mavlink.GSM_LINK_TYPE_NONE,
            255,255,255,255)
        self.verbose = False
        self.srcSystem = 1
        self.srcComponent = 1
        self.fakemav = self.master.mav.__class__(0,self.srcSystem,mavutil.mavlink.MAV_COMP_ID_UDP_BRIDGE)
        self.RSSIreport_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
              ('gsm_dev', str, None)
          ])
        self.add_command('RSSIreport', self.cmd_RSSIreport, "RSSIreport module", ['status','set (LOGSETTING)'])
        self.serCon = None

    def unload(self):
        if self.serCon is not None:
            self.serCon.close()

    def usage(self):
        '''show help on command line options'''
        return "Usage: RSSIreport <status|set>"

    def cmd_RSSIreport(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.RSSIreport_settings.command(args[1:])
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        if self.serCon is not None:
            serial_port = self.serCon.port
        else:
            serial_port = None;
        return("querying modem on %s\nrssi= %d (%f s ago)" % (serial_port, self.gsm_link_status_msg.rssi, time.time()-self.last_report))

    def set_port(self, port):
        if self.serCon is not None:
            self.serCon.close()
        else:
            self.serCon = serial.Serial(baudrate=57600,timeout=1)
        try:
            self.serCon.port = port
            self.serCon.open()
        except Exception as e:
            self.serCon = None;
            raise e
        else:
            pass

    def get_rssi(self):
        self.serCon.write('AT^HCSQ?\r\n')
        self.serCon.flush()
        time.sleep(0.01)
        while self.serCon.inWaiting() > 0:
            ans = self.serCon.readline()
            if ans[0:6]=='^HCSQ:':
                ans = ans[6:-2].split(',',1)
                vals = list()
                if len(ans) > 1:
                    for el in ans[1].split(','):
                        vals.append(int(el))
                vals.extend([255]*(4-len(vals)))
                return [ans[0].strip('"'),vals]
        raise IOError('No HCSQ response from LTE stick')

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_report > self.report_interval:
            self.last_report = now
            if self.serCon is None and self.RSSIreport_settings.get('gsm_dev') is not None:
                try:
                    self.serCon = serial.Serial(port=self.RSSIreport_settings.get('gsm_dev'),baudrate=57600,timeout=1)
                except Exception as e:
                    raise
                    return
            elif self.RSSIreport_settings.get('gsm_dev') is None:
                return

            if self.serCon.port is not self.RSSIreport_settings.get('gsm_dev'):
                self.set_port(self.RSSIreport_settings.get('gsm_dev'))

            rssi = self.get_rssi()

            # status_text_msg = mavutil.mavlink.MAVLink_statustext_message(mavutil.mavlink.MAV_SEVERITY_CRITICAL,'gsm signal strength low!')

            self.gsm_link_status_msg.timestamp = int(self.last_report*1000);
            if rssi[0]=='NOSERVICE':
                self.gsm_link_status_msg.gsm_link_type = mavutil.mavlink.GSM_LINK_TYPE_NONE
            elif rssi[0] == 'GSM':
                self.gsm_link_status_msg.gsm_link_type = mavutil.mavlink.GSM_LINK_TYPE_2G
            elif rssi[0] == 'WCDMA' or rssi[0] == 'TD-SCDMA':
                self.gsm_link_status_msg.gsm_link_type = mavutil.mavlink.GSM_LINK_TYPE_3G
            elif rssi[0] == 'LTE':
                self.gsm_link_status_msg.gsm_link_type = mavutil.mavlink.GSM_LINK_TYPE_4G
            else:
                self.gsm_link_status_msg.gsm_link_type = mavutil.mavlink.GSM_LINK_TYPE_UNKNOWN

            self.gsm_link_status_msg.rssi = rssi[1][0];
            self.gsm_link_status_msg.rsrp_rscp = rssi[1][1];
            self.gsm_link_status_msg.sinr_ecio = rssi[1][2];
            self.gsm_link_status_msg.rsrq = rssi[1][3];

            for output in self.mpstate.mav_outputs:
                output.write(self.gsm_link_status_msg.pack(self.fakemav))
                # if (rssi < 20 and now-self.last_low > self.report_low_interval):
                #     output.write(status_text_msg.pack(self.fakemav))
                #     self.last_low = now

            for master in self.mpstate.mav_master:
                master.write(self.gsm_link_status_msg.pack(self.master.mav))


def init(mpstate):
    '''initialise module'''
    return RSSIreport(mpstate)
