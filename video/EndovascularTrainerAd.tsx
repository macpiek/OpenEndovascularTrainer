import React from 'react';
import {
  AbsoluteFill,
  Easing,
  interpolate,
  Sequence,
  useCurrentFrame,
  useVideoConfig,
} from 'remotion';

const clamp = {
  extrapolateLeft: 'clamp' as const,
  extrapolateRight: 'clamp' as const,
};

const easeOut = Easing.bezier(0.16, 1, 0.3, 1);
const easeInOut = Easing.bezier(0.45, 0, 0.55, 1);

const colors = {
  ink: '#071014',
  panel: 'rgba(9, 23, 31, 0.78)',
  cyan: '#64d8ff',
  mint: '#7cf8c6',
  lime: '#d8f26d',
  orange: '#ffb65c',
  red: '#ff6a7a',
  white: '#f7fbff',
  muted: '#a9bdc7',
};

const fade = (frame: number, start: number, duration: number) =>
  interpolate(frame, [start, start + duration], [0, 1], {
    ...clamp,
    easing: easeOut,
  });

const segment = (frame: number, start: number, end: number, tail = 18) => {
  const enter = fade(frame, start, tail);
  const exit = interpolate(frame, [end - tail, end], [1, 0], {
    ...clamp,
    easing: Easing.in(Easing.cubic),
  });
  return Math.min(enter, exit);
};

const useSceneFrame = (offset = 0) => {
  const frame = useCurrentFrame();
  return frame - offset;
};

const ScreenGrain: React.FC = () => {
  const frame = useCurrentFrame();
  const opacity = 0.055 + Math.sin(frame * 1.7) * 0.015;

  return (
    <AbsoluteFill
      style={{
        opacity,
        backgroundImage:
          'radial-gradient(circle at 18% 26%, rgba(255,255,255,0.55) 0 1px, transparent 1.6px), radial-gradient(circle at 72% 64%, rgba(255,255,255,0.35) 0 1px, transparent 1.5px)',
        backgroundSize: '17px 17px, 23px 23px',
        mixBlendMode: 'screen',
      }}
    />
  );
};

const MonitorLine: React.FC<{top: number; color: string; delay: number}> = ({
  top,
  color,
  delay,
}) => {
  const frame = useCurrentFrame();
  const progress = (frame + delay) % 120;
  const x = interpolate(progress, [0, 119], [-140, 430], clamp);
  const points = Array.from({length: 28}, (_, i) => {
    const px = i * 18;
    const y =
      top +
      Math.sin((i + frame * 0.12 + delay) * 0.85) * 8 +
      (i % 9 === 5 ? -18 : 0);
    return `${px},${y}`;
  }).join(' ');

  return (
    <>
      <polyline
        points={points}
        fill="none"
        stroke={color}
        strokeWidth="3"
        strokeLinecap="round"
        strokeLinejoin="round"
        opacity="0.85"
      />
      <rect x={x} y={top - 38} width="120" height="76" fill="url(#monitorGlow)" />
    </>
  );
};

const PatientMonitor: React.FC<{opacity: number}> = ({opacity}) => {
  return (
    <div
      style={{
        position: 'absolute',
        right: 84,
        top: 110,
        width: 470,
        height: 248,
        border: '1px solid rgba(124,248,198,0.32)',
        background: 'rgba(3, 11, 15, 0.72)',
        boxShadow: '0 26px 80px rgba(0,0,0,0.35), inset 0 0 50px rgba(100,216,255,0.08)',
        opacity,
      }}
    >
      <svg width="470" height="248" viewBox="0 0 470 248">
        <defs>
          <linearGradient id="monitorGlow" x1="0" x2="1">
            <stop offset="0" stopColor="rgba(124,248,198,0)" />
            <stop offset="0.5" stopColor="rgba(124,248,198,0.14)" />
            <stop offset="1" stopColor="rgba(124,248,198,0)" />
          </linearGradient>
        </defs>
        <g opacity="0.2">
          {Array.from({length: 9}, (_, i) => (
            <line key={`h-${i}`} x1="0" x2="470" y1={28 + i * 24} y2={28 + i * 24} stroke="#7cf8c6" />
          ))}
          {Array.from({length: 12}, (_, i) => (
            <line key={`v-${i}`} y1="0" y2="248" x1={28 + i * 38} x2={28 + i * 38} stroke="#64d8ff" />
          ))}
        </g>
        <MonitorLine top={80} color={colors.mint} delay={0} />
        <MonitorLine top={158} color={colors.cyan} delay={37} />
      </svg>
      <div style={{position: 'absolute', left: 22, top: 18, color: colors.muted, font: '600 18px ui-monospace, Menlo, monospace'}}>
        FLUORO TRAINING MODE
      </div>
      <div style={{position: 'absolute', right: 22, bottom: 18, color: colors.lime, font: '700 34px ui-monospace, Menlo, monospace'}}>
        72 bpm
      </div>
    </div>
  );
};

const VesselScene: React.FC<{
  progress: number;
  contrast: number;
  rotation: number;
  scale?: number;
}> = ({progress, contrast, rotation, scale = 1}) => {
  const frame = useCurrentFrame();
  const dash = interpolate(progress, [0, 1], [980, 0], clamp);
  const contrastDash = interpolate(contrast, [0, 1], [720, 0], clamp);
  const scanX = interpolate((frame % 105) / 104, [0, 1], [-100, 1000], clamp);

  return (
    <svg
      width="940"
      height="700"
      viewBox="0 0 940 700"
      style={{
        position: 'absolute',
        left: 475,
        top: 210,
        transform: `translate(-50%, -50%) scale(${scale}) rotate(${rotation}deg)`,
        overflow: 'visible',
        filter: 'drop-shadow(0 28px 72px rgba(0,0,0,0.44))',
      }}
    >
      <defs>
        <filter id="softXray">
          <feGaussianBlur stdDeviation="1.2" />
        </filter>
        <linearGradient id="wireGradient" x1="0" x2="1">
          <stop offset="0" stopColor="#fff" />
          <stop offset="0.52" stopColor="#dde8ed" />
          <stop offset="1" stopColor="#7cf8c6" />
        </linearGradient>
        <linearGradient id="contrastGradient" x1="0" x2="1">
          <stop offset="0" stopColor="#fff" stopOpacity="0" />
          <stop offset="0.2" stopColor="#fff" stopOpacity="0.95" />
          <stop offset="0.82" stopColor="#64d8ff" stopOpacity="0.55" />
          <stop offset="1" stopColor="#64d8ff" stopOpacity="0" />
        </linearGradient>
      </defs>
      <g opacity="0.22" filter="url(#softXray)">
        <path d="M162 554 C270 495 348 434 426 342 C502 252 604 228 762 170" stroke="#cbd5d9" strokeWidth="82" fill="none" strokeLinecap="round" />
        <path d="M430 345 C522 364 638 416 782 522" stroke="#cbd5d9" strokeWidth="58" fill="none" strokeLinecap="round" />
        <path d="M428 344 C350 278 270 218 178 162" stroke="#cbd5d9" strokeWidth="46" fill="none" strokeLinecap="round" />
      </g>
      <g opacity="0.34">
        <path d="M162 554 C270 495 348 434 426 342 C502 252 604 228 762 170" stroke="#fff" strokeWidth="8" fill="none" strokeLinecap="round" />
        <path d="M430 345 C522 364 638 416 782 522" stroke="#fff" strokeWidth="6" fill="none" strokeLinecap="round" />
        <path d="M428 344 C350 278 270 218 178 162" stroke="#fff" strokeWidth="5" fill="none" strokeLinecap="round" />
      </g>
      <g opacity="0.82">
        <path
          d="M106 606 C212 540 316 462 426 342 C502 252 604 228 762 170"
          stroke="url(#wireGradient)"
          strokeWidth="13"
          fill="none"
          strokeLinecap="round"
          strokeDasharray="980"
          strokeDashoffset={dash}
        />
        <circle
          cx={interpolate(progress, [0, 1], [106, 762], clamp)}
          cy={interpolate(progress, [0, 1], [606, 170], clamp)}
          r="13"
          fill="#ffffff"
          opacity={progress > 0.02 ? 0.95 : 0}
        />
      </g>
      <g opacity={interpolate(contrast, [0, 0.18, 1], [0, 1, 0.9], clamp)}>
        <path
          d="M426 342 C502 252 604 228 762 170"
          stroke="url(#contrastGradient)"
          strokeWidth="54"
          fill="none"
          strokeLinecap="round"
          strokeDasharray="720"
          strokeDashoffset={contrastDash}
        />
        <path
          d="M430 345 C522 364 638 416 782 522"
          stroke="url(#contrastGradient)"
          strokeWidth="42"
          fill="none"
          strokeLinecap="round"
          strokeDasharray="690"
          strokeDashoffset={contrastDash + 120}
        />
      </g>
      <rect x={scanX} y="35" width="76" height="610" fill="rgba(255,255,255,0.06)" />
    </svg>
  );
};

const Carm: React.FC<{opacity: number; angle: number}> = ({opacity, angle}) => (
  <div
    style={{
      position: 'absolute',
      left: 1030,
      top: 300,
      width: 580,
      height: 520,
      opacity,
      transform: `rotate(${angle}deg)`,
      transformOrigin: '52% 48%',
    }}
  >
    <svg width="580" height="520" viewBox="0 0 580 520">
      <path d="M405 68 C532 132 546 350 407 421" fill="none" stroke="rgba(247,251,255,0.28)" strokeWidth="42" strokeLinecap="round" />
      <rect x="348" y="26" width="144" height="84" rx="10" fill="rgba(247,251,255,0.18)" stroke="rgba(255,255,255,0.38)" />
      <rect x="345" y="391" width="150" height="88" rx="10" fill="rgba(247,251,255,0.16)" stroke="rgba(255,255,255,0.36)" />
      <line x1="318" y1="110" x2="188" y2="360" stroke="rgba(100,216,255,0.22)" strokeWidth="2" />
      <line x1="490" y1="110" x2="316" y2="360" stroke="rgba(100,216,255,0.22)" strokeWidth="2" />
      <rect x="72" y="356" width="290" height="35" rx="7" fill="rgba(255,255,255,0.16)" />
      <rect x="134" y="391" width="168" height="24" fill="rgba(255,255,255,0.11)" />
      <text x="355" y="509" fill={colors.muted} fontFamily="ui-monospace, Menlo, monospace" fontSize="21">
        C-ARM CONTROL
      </text>
    </svg>
  </div>
);

const CopyBlock: React.FC<{
  opacity: number;
  kicker: string;
  title: string;
  body: string;
  accent?: string;
  top?: number;
}> = ({opacity, kicker, title, body, accent = colors.cyan, top = 330}) => (
  <div
    style={{
      position: 'absolute',
      left: 96,
      top,
      width: 760,
      opacity,
      transform: `translateY(${interpolate(opacity, [0, 1], [34, 0], clamp)}px)`,
    }}
  >
    <div style={{font: '700 24px Inter, Arial, sans-serif', color: accent, marginBottom: 20}}>
      {kicker}
    </div>
    <div style={{font: '800 86px Inter, Arial, sans-serif', lineHeight: 0.95, color: colors.white, marginBottom: 26}}>
      {title}
    </div>
    <div style={{font: '500 34px Inter, Arial, sans-serif', lineHeight: 1.22, color: colors.muted}}>
      {body}
    </div>
  </div>
);

const FeatureRail: React.FC<{opacity: number}> = ({opacity}) => {
  const items = [
    ['PBD guidewire', 'sprężysty prowadnik i kontakt ze ścianą naczynia', colors.mint],
    ['Kontrast', 'przepływ, washout i refluks przy iniekcji', colors.cyan],
    ['C-arm', 'zmiana projekcji bez opuszczania sceny', colors.orange],
  ];

  return (
    <div
      style={{
        position: 'absolute',
        left: 96,
        bottom: 74,
        display: 'grid',
        gridTemplateColumns: 'repeat(3, 1fr)',
        gap: 20,
        width: 1120,
        opacity,
      }}
    >
      {items.map(([title, body, color]) => (
        <div
          key={title}
          style={{
            height: 132,
            border: '1px solid rgba(255,255,255,0.16)',
            background: 'rgba(5, 18, 24, 0.64)',
            padding: '24px 26px',
          }}
        >
          <div style={{height: 5, width: 72, background: color, marginBottom: 18}} />
          <div style={{font: '800 25px Inter, Arial, sans-serif', color: colors.white, marginBottom: 8}}>
            {title}
          </div>
          <div style={{font: '500 19px Inter, Arial, sans-serif', lineHeight: 1.24, color: colors.muted}}>
            {body}
          </div>
        </div>
      ))}
    </div>
  );
};

const Background: React.FC = () => {
  const frame = useCurrentFrame();
  const slow = frame / 540;
  return (
    <AbsoluteFill
      style={{
        background:
          'linear-gradient(118deg, #020607 0%, #0d1820 38%, #132526 65%, #090c0f 100%)',
        overflow: 'hidden',
        fontFamily: 'Inter, Arial, sans-serif',
      }}
    >
      <div
        style={{
          position: 'absolute',
          inset: 0,
          backgroundImage:
            'linear-gradient(rgba(100,216,255,0.05) 1px, transparent 1px), linear-gradient(90deg, rgba(100,216,255,0.04) 1px, transparent 1px)',
          backgroundSize: '72px 72px',
          transform: `translate(${interpolate(slow, [0, 1], [0, -72])}px, ${interpolate(slow, [0, 1], [0, -44])}px)`,
        }}
      />
      <div
        style={{
          position: 'absolute',
          inset: 0,
          background:
            'radial-gradient(circle at 72% 44%, rgba(124,248,198,0.18), transparent 34%), radial-gradient(circle at 18% 85%, rgba(255,182,92,0.1), transparent 30%)',
        }}
      />
      <ScreenGrain />
    </AbsoluteFill>
  );
};

export const EndovascularTrainerAd: React.FC = () => {
  const frame = useCurrentFrame();
  const {fps} = useVideoConfig();
  const titleOpacity = segment(frame, 0, 126, 24);
  const procedureOpacity = segment(frame, 96, 286, 24);
  const featureOpacity = segment(frame, 234, 424, 26);
  const finalOpacity = fade(frame, 402, 32);

  const wireProgress = interpolate(frame, [56, 230], [0, 1], {
    ...clamp,
    easing: easeInOut,
  });
  const contrastProgress = interpolate(frame, [228, 370], [0, 1], {
    ...clamp,
    easing: easeOut,
  });
  const carmAngle = interpolate(frame, [120, 400], [-8, 12], {
    ...clamp,
    easing: easeInOut,
  });

  return (
    <AbsoluteFill>
      <Background />
      <div
        style={{
          position: 'absolute',
          left: 72,
          top: 58,
          color: colors.white,
          font: '800 28px Inter, Arial, sans-serif',
          opacity: 0.92,
        }}
      >
        Open Endovascular Trainer
      </div>
      <div
        style={{
          position: 'absolute',
          right: 78,
          top: 58,
          color: colors.muted,
          font: '600 20px ui-monospace, Menlo, monospace',
          opacity: 0.78,
        }}
      >
        BROWSER-BASED SIMULATION / {Math.floor(frame / fps).toString().padStart(2, '0')}s
      </div>

      <VesselScene
        progress={wireProgress}
        contrast={contrastProgress}
        rotation={interpolate(frame, [0, 540], [-7, 3], clamp)}
        scale={interpolate(frame, [0, 540], [1.05, 1.16], clamp)}
      />
      <Carm opacity={procedureOpacity * 0.9 + featureOpacity * 0.72} angle={carmAngle} />
      <PatientMonitor opacity={procedureOpacity * 0.9 + featureOpacity * 0.55} />

      <CopyBlock
        opacity={titleOpacity}
        kicker="SYMULATOR ENDOWASKULARNY"
        title="Trenuj procedurę, zanim wejdziesz na salę."
        body="Realistyczny prowadnik, fluoroskopia i kontrast w interaktywnym środowisku."
        accent={colors.mint}
      />

      <Sequence from={106}>
        <CopyBlock
          opacity={procedureOpacity}
          kicker="PRECYZJA POD KONTROLĄ"
          title="Nauka ruchu, obrazu i decyzji."
          body="Ćwicz prowadzenie narzędzi, projekcje aparatu i ocenę przepływu kontrastu."
          accent={colors.cyan}
          top={346}
        />
      </Sequence>

      <FeatureRail opacity={featureOpacity} />

      <div
        style={{
          position: 'absolute',
          left: 96,
          top: 330,
          width: 830,
          opacity: finalOpacity,
          transform: `translateY(${interpolate(finalOpacity, [0, 1], [42, 0], clamp)}px)`,
        }}
      >
        <div style={{font: '800 96px Inter, Arial, sans-serif', lineHeight: 0.95, color: colors.white, marginBottom: 28}}>
          Open Endovascular Trainer
        </div>
        <div style={{font: '600 38px Inter, Arial, sans-serif', lineHeight: 1.18, color: colors.muted, marginBottom: 40}}>
          Proceduralny trening endowaskularny dostępny z poziomu przeglądarki.
        </div>
        <div
          style={{
            display: 'inline-flex',
            alignItems: 'center',
            gap: 18,
            border: `2px solid ${colors.mint}`,
            color: colors.white,
            padding: '18px 26px',
            font: '800 26px Inter, Arial, sans-serif',
            background: 'rgba(124,248,198,0.09)',
          }}
        >
          URUCHOM SYMULATOR
          <span style={{color: colors.mint}}>→</span>
        </div>
      </div>
      <div
        style={{
          position: 'absolute',
          left: 0,
          right: 0,
          bottom: 0,
          height: 10,
          background: `linear-gradient(90deg, ${colors.mint}, ${colors.cyan}, ${colors.orange})`,
          transform: `scaleX(${interpolate(frame, [0, 539], [0, 1], clamp)})`,
          transformOrigin: 'left',
        }}
      />
    </AbsoluteFill>
  );
};
