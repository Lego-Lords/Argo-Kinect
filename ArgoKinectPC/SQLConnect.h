#pragma once
#ifndef SQLCONNECT_H
#define SQLCONNECT_H

#include <mysql.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

class SQLConnect
{
private: 
	int modelSelected;
	int currentStep;
	int maxStep;
	//add method skeleton

public:
	SQLConnect();
	MYSQL * setUpConnection(char* server, char* username, char* password, char* db);
	MYSQL* SQLConnect::mysql_connection_setup(struct connection_details mysql_details);
	MYSQL_RES * mysql_perform_query(MYSQL * connection, char * sql_query);
	int getSelectedModel(MYSQL *connection);
	int getCurrentStep(MYSQL *connection);
	int getMaxStep(MYSQL *connection);
	void updateNextStep(MYSQL *connection, int nextStep);
	void updateModelSelected(MYSQL *connection, int modelSelected);
	void updateHasError(MYSQL *connection, int hasError);
	void setRotValue(MYSQL * connection, int rotValue);

};

#endif