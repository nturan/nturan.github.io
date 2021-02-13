simPhys = {
  step: 1000/60,
  boxSize: {width: 500, height: 500},
  gravity: 9.81,
  
  
  loopPID: function(setPoint, measuredValue,
                     integralE, prevError, step,
                     coeff={kP: 0.025, kI: 0.2E-5, kD: 4}, tol=1.0E-5){
    let ERR = setPoint - measuredValue;
    if(Math.abs(prevError)<tol){
      prevError = 0;
    }
    if(Math.abs(ERR)<tol){
      ERR = 0;
    }
    let integral = integralE;
    if(integral >=1.0E+3){
//       integral = 1.0E+3;
    }else if(Math.abs(ERR)< 1.0E-03){
//       integral = 0;
    }else{
      integral += ERR * step; 
    }
    let derivative = (ERR - prevError) / step;
    let output = coeff.kP * ERR + coeff.kI*integral + coeff.kD * derivative;

    return {c: output, er: ERR, integ: integral};
  },

  airResistance: function(velocity, res){
    return  {x: -velocity.x*res, 
             y: -velocity.y*res, 
             z: -velocity.z*res};
  }
  
  

};
