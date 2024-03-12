using Plots
using SmoothLivePlot


#gears
g0 = 1
g1 = 0.1
g2 = 0.5
g3 = 3
g4 = 6


#constants
n = 1.3 #polytropic index 
P1 = 100000 #pressure that accounts for temperature during compression [Pa]
P2 = 10300000 #pressure that accounts for temperature during expansion [Pa]


#variables
R = 0.3 #flywheel radius [m]
m = 70 #mass of the flywheel [kg]
I = m/2*R^2 #moment of inertia of flywheel [kg*m^2]
Rp = 0.025 #piston radius [m]
A = pi * Rp^2 #piston area [m]
theta = 0 #crank angle [rad]
x = 0.05*2*R #clearance distance [m]
V1 = (R - R*cos(pi)+x) #volume when piston down [m^3]
V2 = (R - R*cos(0)+x) #volume when piston up [m^3]
C = (x/(2*R+x))^n #load constant [kg*m^2]
angVel = 50 #angular velocity [rad/s]
timeStep = 0.001

gear = []
push!(gear, g0)
timeOfSim = [] 
thetaVec = []
angVelVec = []
simTime = []
ACCvec = []

function angAcc_equation(angVelprev, thetaprev, gearC)
    halfcircles = trunc(Int, thetaprev / pi)
    halfcircles_is_odd = isodd(halfcircles)

    if halfcircles_is_odd
        angVelPrim = gearC * (P2 * ((V1 / (R - R * cos(mod(thetaprev, pi)) + x))^n) * (A * R * sin(mod(thetaprev, pi))) / I - (C * angVelprev * angVelprev) / I)
    else
        angVelPrim = gearC * (P1 * ((V2 / (R - R * cos(mod(thetaprev, pi)) + x))^n) * (A * R * sin(mod(thetaprev, pi))) / I - (C * angVelprev * angVelprev) / I)
    end

    return angVelPrim
end

function euler()
    empty!(angVelVec)
    empty!(thetaVec)
    empty!(simTime)

    push!(thetaVec, theta)
    push!(angVelVec, angVel)
    push!(simTime, 0)
    
    newTime = 0
    d = 1

    for j in 1:length(timeOfSim)
        timeOfSimButLoop = trunc(Int, timeOfSim[j] / timeStep)

        for i in 2:timeOfSimButLoop
            angAcc = angAcc_equation(angVelVec[end], thetaVec[end], gear[d])
            newAngVel = angVelVec[end] + timeStep * angAcc
            newtheta = thetaVec[end] + timeStep * newAngVel

            angVel = newAngVel

            push!(angVelVec, newAngVel)
            push!(thetaVec, newtheta)
            newTime = newTime + timeStep
            push!(simTime, newTime)
        end

        d += 1
    end
end

function gear_change()
    gearA = 0  # Dodajemy deklarację zmiennej przed blokiem try
    
    print("Which gear would you like to change to? [1/2/3/4]: ")
    input = readline()
    
    if strip(input) != ""
        try
            gearA = parse(Int, strip(input))
            println("Selected gear = ", gearA)
        catch e
            println("Parse error: ", e)
            return gear_change()  
        end
    else
        println("No value has been entered.")
        return gear_change() 
    end
    
    if gearA == 1
        gear = g1
    elseif gearA == 2
        gear = g2
    elseif gearA == 3
        gear = g3
    elseif gearA == 4
        gear = g4
    else
        println("Wrong gear value")
        return gear_change()  
    end
    
    return gear
end

function gear_time()
    print("How long would you like to simulate on selected gear?: ")
    gearTime = 0
    input = readline()
    if strip(input) != ""
        try
            gearTime = parse(Int, strip(input))
            println("Selected gear time = ", gearTime, " seconds")
        catch e
            println("Parse error: ", e)
        end
    else
        println("No value has been entered.")
    end
    return gearTime
end

print("Without changing gears engine should work: ")
inputStr = readline()
if strip(inputStr) != ""
    try
        timeOfSimNoGear = parse(Int, strip(inputStr))
        println("No gear time = ", timeOfSim, " seconds.")
        push!(timeOfSim, timeOfSimNoGear)
    catch e
        println("Parse error: ", e)
    end
else
    println("No value has been entered.")
end

while true
    println("Would you like to change gear? [y/n]")
    input = readline()

    if input == "y"
        gearB = gear_change()
        gearTime = gear_time()
        push!(gear, gearB)
        push!(timeOfSim, gearTime)
        
    elseif input == "n"
        println("Running simulation")
        break
    
    else 
        println("Wrong value has been entered.")
    end
end

euler()
gr(size=(800, 450))
plot(simTime, angVelVec, title="angular velocity over time", label="rad/s")
