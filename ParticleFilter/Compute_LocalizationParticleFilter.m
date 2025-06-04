function particles = Compute_LocalizationParticleFilter(robot,laser,mapa,particles)



%% INITIALIZATIONS

% Normalize laser observations
laser.rangefinder        = laser.rangefinder / max(laser.rangefinder);

% Creta a robot variable for the partciles
particlesiclerobot.radio      = robot.radio;
particlesiclerobot.pose       = [];

% Create a laser variable for the particles
laserparticlesicle.maxdis     = laser.maxdis;
laserparticlesicle.angle      = laser.angle;
laserparticlesicle.nbeams     = laser.nbeams;
laserparticlesicle.noisesigma = laser.noisesigma;


%% PREDICTION STEP: Apply motion model (odometry) to each particle

Xpredi = particles.X + robot.odometria(1) + 1*randn(size(particles.X));
Ypredi = particles.Y + robot.odometria(2) + 1*randn(size(particles.Y));
Tpredi = particles.T + robot.odometria(3) + 0.1*randn(size(particles.T));
Tpredi = AjustaAngulo(Tpredi);



%% EVALUATE IMPORTANCE WEIGHTS

for ipar=1:particles.NP

    % i) Compute laser measurements for each particle
    particlesiclerobot.pose = [Xpredi(ipar), Ypredi(ipar), Tpredi(ipar)];
    particlesiclelaser.rangefinder  = Compute_RangeFinder(particlesiclerobot, laserparticlesicle, mapa, false);

    % Normalize laser observations
    if sum(particlesiclelaser.rangefinder)~=0
        particlesiclelaser.rangefinder = particlesiclelaser.rangefinder / max(particlesiclelaser.rangefinder);
    end

    % ii) Compute weight of each particlesicle
    particles.W(ipar) = exp(-0.5 *(laser.rangefinder-particlesiclelaser.rangefinder)*(laser.rangefinder-particlesiclelaser.rangefinder)');
end

% iii) Normalize weihgts
particles.W = particles.W/sum(particles.W);



%% SELECTION STEP

% Minimum variance resampling
outIndex = deterministicR(1:particles.NP,particles.W);

% Keep particlesicles with resampled indices
particles.X = Xpredi(outIndex);
particles.Y = Ypredi(outIndex);
particles.T = Tpredi(outIndex);