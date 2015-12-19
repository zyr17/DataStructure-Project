#ifndef __TAXI_DATA_H__
#define __TAXI_DATA_H__
struct Taxi_Data{
	int id;
	int time;
	int alert;
	double lat, lon;
	double x, y;
	int empty;
	char light;
	int high;
	int brake;
	double speed;
	double angle;
	int gps;
	Taxi_Data(){}
	Taxi_Data(int id, int time, int alert, double lon, double lat, double x, double y, int empty, char light, int high, int brake, double speed, double angle, int gps) :
		id(id), time(time), alert(alert), lon(lon), lat(lat), x(x), y(y), empty(empty), light(light), high(high), brake(brake), speed(speed), angle(angle), gps(gps) {}
	bool operator< (Taxi_Data k) const{
		return time < k.time;
	}
};
#endif