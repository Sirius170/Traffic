function initializeOTLScenario(s)
% Register finite actor poses before restoring scheduled visibility.
% R2025b actorPoses emits NaNs for the initially hidden OTL actors.
% This does not advance time or change EntryTime, positions, or control.
% Call the R2025b hidden method directly: do not silently skip setup.
actors = s.Actors;
visible = false(size(actors));
for k = 1:numel(actors)
    visible(k) = actors(k).IsVisible;
end
restoreVisibility = onCleanup(@restoreActors);
for k = 1:numel(actors)
    actors(k).IsVisible = true;
end
poses = actorPoses(s);
fields = {'Position','Velocity','Roll','Pitch','Yaw','AngularVelocity'};
for k = 1:numel(poses)
    for j = 1:numel(fields)
        value = poses(k).(fields{j});
        if any(~isfinite(value(:)))
            error('OpenTrafficLab:InvalidPose', ...
                'Actor %d has invalid %s before initialization.', ...
                poses(k).ActorID,fields{j});
        end
    end
end
s.setUpSensorSimulation();
clear restoreVisibility
assert(s.SensorSimulation.SceneCreated, ...
    'OpenTrafficLab:SceneNotCreated','Sensor scene initialization did not persist.');
fprintf('OTL: sensor scene initialized for %d actors.\n',numel(actors));

    function restoreActors()
        for idx = 1:numel(actors)
            actors(idx).IsVisible = visible(idx);
        end
    end
end
