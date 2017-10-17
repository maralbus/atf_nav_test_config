#!/usr/bin/python
import os
import sys
import csv
import unittest
import rospy
import rosunit
from datetime import datetime

ROW_LENGTH = 14                 ## count for the number of column, categories.csv file has
CONTACT_INDEX = 8               ## column index for the phone number, it has total 6 values  
NAV_GOAL_INDEX = 7              ## column index for the value of Zuordnung zu Station
LEVEL1_NAME_INDEX = 0           ## column index for the value of LEVEL_1_NAME
CATEGORY_LEVEL1_ID_INDEX = 1    ## column index for the value of CATEGORY_LEVEL_1_PIM_ID
SPRACHAUSGABE_INDEX = 3         ## column index for the value of Sprachausgabe
LEVEL1_CONTACT_INDEX = 8        ## column index for the first contact number


class csv_checker(object):
    """
    Instances of this class can be used to check the given CSV file
    """
    def __init__(self):
        """
        Instantiate a `csv_checker`, supplying expected `csv_file_path`
        """
        self._csv_file_name = ""
        self._record_invalid_row_check = {}
        self._record_length_checks = {}
        self._record_missing_entries_check = {}
        self._record_typos_checks = {}
        self._invalid_row_error = False
        self._length_check_error = False
        self._missing_entries = False
        self._row_typos = False
        self._nav_goals = {}
    
    def set_params(self):
        rospy.init_node("csv_checker")
        self._csv_file_name = rospy.get_param('~categories_file')
        self._nav_goals = rospy.get_param('/script_server/base')
        print "checking file: ", self._csv_file_name    
        print "nav goals: ", self._nav_goals.keys()
        # run the checker
        if self._csv_file_name:
            print "running checker ..."
            self.file_reader()

    def run_checker(self):
        """
        Apply all the check method for error
        """
        has_error = False
        if self._invalid_row_error:
            print "\n\n\tFound invalid row in CSV file!"
            self.display_error_summery(self._record_invalid_row_check)
        if self._length_check_error:
            print "\n\n\tFound invalid number of columns in CSV file!"
            self.display_error_summery(self._record_length_checks)
        if self._missing_entries:
            print "\n\n\tFound missing entries from row in CSV file!"
            self.display_error_summery(self._record_missing_entries_check)
        if self._row_typos:
            print "\n\n\tFound invalid type in CSV file!"
            self.display_error_summery(self._record_typos_checks)
        if (
                not self._invalid_row_error and 
                not self._length_check_error and
                not self._missing_entries and 
                not self._row_typos
           ):
            has_error = True
            print "\n\tThere is no error in the file! \n"
        return has_error
    
    def display_error_summery(self, error_detail_store={}):
        """
        Validation report, print all the rows with error
        """
        print "\t-----------"
        for key, value in error_detail_store.items():
            print "Row number - ", int(key)+1, ":", value
        return

    
    def file_reader(self):
        """
        Read the file from the given path and perform all checks
        """
        with open(self._csv_file_name, 'rb') as filehandle:
            reader = csv.reader(filehandle, delimiter=';')
            self._invalid_row_error = False
            self._length_check_error = False
            self._row_typos = False
            line_num = 0
            i = 0
            for row in reader:
                try:
                    if i is not 0:
                        if self.check_valid_row(line_num, row):
                            if len(row) > CONTACT_INDEX:
                                self.check_row_typos(line_num,row)
                                self.check_missing_entries(line_num, row)
                    i+=1
                    line_num+=1
                except Exception as e:
                    print 'Exception in file_reader: ',e, 'line:',i+1
                    continue
        return


    def check_valid_row(self, line_num, row):
        """
        For a given row in CSV, check if the row is valid
        """
        is_row_valid = False
        if row:
            is_row_valid = True
            if len(row) != ROW_LENGTH:
                self._length_check_error = True
                self._record_length_checks[line_num] = row
        else:
            self._invalid_row_error = True
            is_row_valid= False
            self._record_invalid_row_check[line_num] = row
        return is_row_valid
            


    def check_row_typos(self, line_num, row):
        """
        For a given row check if there are any typos in the fields nav_goal and level_1_phone_number
        """
        if row[NAV_GOAL_INDEX]:
            if (' ' in str(row[NAV_GOAL_INDEX])) or (str(row[NAV_GOAL_INDEX]) not in self._nav_goals.keys()):
                self._row_typos = True
                self._record_typos_checks[line_num] = "[Zuordnung zu Station] : " + str(row)
                return
        else:
            self._missing_entries = True
            self._record_missing_entries_check[line_num] = row
        if row[CONTACT_INDEX]:
            for i in range(0, 6):
                if row[CONTACT_INDEX+i] != '':
                    if not row[CONTACT_INDEX+i].isdigit():
                        phone_number = row[CONTACT_INDEX+i]
                        if phone_number[0] == "+": ## only string allowed in phone number is a leading plus sign, otherwise invalid
                            phone_number = phone_number[1:]
                            if not phone_number.isdigit():
                                self._row_typos = True
                                self._record_typos_checks[line_num] = "[Telefonkontakt1] : " + str(row)
                                return
                        else:
                            self._row_typos = True
                            self._record_typos_checks[line_num] = "[Telefonkontakt1] : " + str(row)
                            return
        else:
            self._missing_entries = True
            self._record_missing_entries_check[line_num] = row
        return


    def check_missing_entries(self, line_num, row):
        """
        For a given row check if mandatory fields are blank
        """
        if (
                row[LEVEL1_NAME_INDEX] == '' or 
                row[CATEGORY_LEVEL1_ID_INDEX] == '' or 
                row[SPRACHAUSGABE_INDEX] == '' or 
                row[NAV_GOAL_INDEX] == '' or 
                row[LEVEL1_CONTACT_INDEX] == ''
           ):
            self._missing_entries = True
            self._record_missing_entries_check[line_num] = row
            
    def has_filename(self):
        return True if self._csv_file_name != "" else False
    
    def has_navigation_goals(self):
        return True if len(self._nav_goals) != 0 else False
            

class TestCSVChecker(unittest.TestCase):
    
    def test_checker(self):
        _csv_checker =  None
        _csv_checker = csv_checker()
        # test object creation
        if not _csv_checker:
            self.fail('csv_checker could not be created')
            return
        
        _csv_checker.set_params()
        
        # test file name 
        if not _csv_checker.has_filename():
            self.fail('unable to fetch CSV file name')
        # test for nav goals
        if not _csv_checker.has_navigation_goals():
            self.fail('unable to fetch navigation goals')
        # test csv_checker success
        if not _csv_checker.run_checker():
            self.fail('found error in CSV file (%s)'%_csv_checker._csv_file_name)

if __name__ == "__main__":
    try:
        rosunit.unitrun('msh_env_config', 'testcsv_checker', TestCSVChecker)
    except rospy.ROSInterruptException: pass

