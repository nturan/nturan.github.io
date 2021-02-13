integrator = {

  rk4: function (fun, x, t, step, arg) {
    
    let k1 = fun(t, x, arg);
//    console.log(k1);
//    console.log(x);
    let k2 = fun(t+step/2, math.add(x, math.multiply(step/2, k1)), arg);
    let k3 = fun(t+step/2, math.add(x, math.multiply(step/2, k2)), arg);
    let k4 = fun(t+step,   math.add(x, math.multiply(step,   k3)), arg);
    //new stateVector
    return math.add(x, math.multiply(step/6, math.add(k1,
                                                      math.multiply(2, k2),
                                                      math.multiply(2, k3),
                                                      k4)));

  }

}
