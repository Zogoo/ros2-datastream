import layout from '../../shared/onsen_layout.json';
import robotSpec from '../../shared/robot_spec.json';
import objectProfiles from '../../shared/object_profiles.json';

const REALISM = (typeof window !== 'undefined' && window.REALISM_PROFILE) || 'low';

export const config = {
  layout,
  robot: robotSpec,
  objects: objectProfiles,
  realism: REALISM,
  physicsHz: 60,
  rosUrl: typeof window !== 'undefined'
    ? `ws://${window.location.hostname}:9090`
    : 'ws://localhost:9090',
};
