import { makeStyles } from '@material-ui/core/styles';
import React from 'react';

interface TargetIndicatorProps {
  x: number;
  y: number;
  // Add visibility?
}

const useStyles = makeStyles({
  // style rule
  indicator: {
    position: 'absolute',
    borderRadius: '50%',
    borderColor: '#A6E22E',
    borderWidth: '2px',
    borderStyle: 'solid',
    mixBlendMode: 'hard-light',
    //backgroundColor: props.hasControl ? '#9acd32' : props.on ? '#eb6534' : '#888', // Green if has control, blue if on the robot, grey otherwise
    //left: `${props.x - size / 2}px`,
    //top: `${props.y - size / 2}px`,
    //color: '#eee',
    //height: `${size}px`,
    //width: `${size}px`,
    //fontSize: '16px',
    //textAlign: 'center',
  },
  indicatorLarge: (props: TargetIndicatorProps) => {
    const largeSize = 30;
    return {
      left: `${props.x - largeSize / 2}px`,
      top: `${props.y - largeSize / 2}px`,
      width: `${largeSize}px`,
      height: `${largeSize}px`,
    };
  },
  indicatorSmall: (props: TargetIndicatorProps) => {
    const smallSize = 16;
    return {
      left: `${props.x - smallSize / 2}px`,
      top: `${props.y - smallSize / 2}px`,
      width: `${smallSize}px`,
      height: `${smallSize}px`,
    };
  },

  // bar: {
  //   // CSS property
  //   color: props => props.color,
  // },
});

export default function TargetIndicator(props: TargetIndicatorProps) {
  const classes = useStyles(props);
  return (
    <>
      <div className={`${classes.indicator} ${classes.indicatorLarge}`}></div>
      <div className={`${classes.indicator} ${classes.indicatorSmall}`}></div>
    </>
  );
}
