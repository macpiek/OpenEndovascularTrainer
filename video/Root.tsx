import {Composition} from 'remotion';
import {EndovascularTrainerAd} from './EndovascularTrainerAd';

export const RemotionRoot = () => {
  return (
    <Composition
      id="EndovascularTrainerAd"
      component={EndovascularTrainerAd}
      durationInFrames={540}
      fps={30}
      width={1920}
      height={1080}
    />
  );
};
