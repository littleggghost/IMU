package gsensor

import (
	"fmt"
	"math"
)

type Driver struct {
	T        float64
	wGyro    float64
	PreRxEst float64
	PreRyEst float64
	PreRzEst float64
}

func NewDriver() *Driver {
	return &Driver{}
}

//simplified kalman filter
//Rx = (AdcRx*Vref/1023 - VzeroG) / Sensitivity, ...
func (d *Driver) AccGyro(Rx, Ry, Rz, RateAxz, RateAyz float64) (RxEst, RyEst, RzEst float64, err error) {
	//accelerometer
	RxAcc := Rx
	RyAcc := Ry
	RzAcc := Rz
	Racc := math.Sqrt(math.Pow(RxAcc, 2) + math.Pow(RyAcc, 2) + math.Pow(RzAcc, 2))
	RxAcc = RxAcc / Racc
	RyAcc = RyAcc / Racc
	RzAcc = RzAcc / Racc
	//gyroscope
	PreAxz := math.Atan2(d.PreRxEst, d.PreRzEst)
	PreAyz := math.Atan2(d.PreRyEst, d.PreRzEst)
	Axz := PreAxz + RateAxz*d.T //Axz := PreAxz + RateAxzAvg*T is more precise, RateAxzAvg = (RateAxz+ PreRateAxz) / 2
	Ayz := PreAyz + RateAyz*d.T
	RxGyro := math.Sin(Axz) / math.Sqrt(1+math.Pow(math.Cos(Axz), 2)*math.Pow(math.Tan(Ayz), 2)) //assume that SQRT(RxGyro^2 + RyGyro^2 + RzGyro^2) = |Rgyro| = 1
	RyGyro := math.Sin(Ayz) / math.Sqrt(1+math.Pow(math.Cos(Ayz), 2)*math.Pow(math.Tan(Axz), 2))
	SignRzGyro := 1.0
	if d.PreRzEst >= 0 {
		SignRzGyro = 1.0
	} else {
		SignRzGyro = -1.0
	}
	RzGyro := SignRzGyro * math.Sqrt(1-math.Pow(RxGyro, 2)-math.Pow(RyGyro, 2)) //or Sign(RzGyro) = Sign(PreRzEst)
	//Combining accelerometer and gyroscope data
	RxEst = (RxAcc + RxGyro*d.wGyro) / (1 + d.wGyro) //wGyro tells us how much we trust our gyro compared to our accelerometer
	RyEst = (RyAcc + RyGyro*d.wGyro) / (1 + d.wGyro)
	RzEst = (RzAcc + RzGyro*d.wGyro) / (1 + d.wGyro)
	R := math.Sqrt(math.Pow(RxEst, 2) + math.Pow(RyEst, 2) + math.Pow(RzEst, 2))
	RxEst = RxEst / R
	RyEst = RyEst / R
	RzEst = RzEst / R
	d.PreRxEst = RxEst
	d.PreRyEst = RyEst
	d.PreRzEst = RzEst
	return
}

func (d *Driver) Acc() (Rx, Ry, Rz float64, err error) {
	return
}

func (d *Driver) Gyro() (RateAxz, RateAyz, RateAxy float64, err error) {
	return
}

func (d *Driver) Loop() (err error) {
	d.T = 0.05  //ms
	d.wGyro = 7 //5~20, maybe it is good
	Rx, Ry, Rz, err := d.Acc()
	d.PreRxEst, d.PreRyEst, d.PreRzEst = Rx, Ry, Rz
	RateAxz, RateAyz, _, err := d.Gyro()
	for {
		RxEst, RyEst, RzEst, err := d.AccGyro(Rx, Ry, Rz, RateAxz, RateAyz)
		fmt.Println(RxEst, RyEst, RzEst)
	}
	return
}
