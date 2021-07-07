declare module 'react-native-compass-heading' {
  export const start: (
    threshold: number,
    callback: ({heading, accuracy, trueHeading, magneticHeading, deviceOrientation, osVersion, deviceModel}) => void,
  ) => Promise<boolean>;

  export const stop: () => Promise<void>;

  export const hasCompass: () => Promise<boolean>;
}
