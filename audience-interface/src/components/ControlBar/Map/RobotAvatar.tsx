import React from 'react';
import { createStyles, makeStyles, Theme } from '@material-ui/core/styles';
//import PriorityHighIcon from '@material-ui/icons/PriorityHigh';
import StarIcon from '@material-ui/icons/Star';
import Lock from '@material-ui/icons/Lock';
//import useVideoContext from '../../../hooks/useVideoContext/useVideoContext';

interface RobotAvatarProps {
  id: number;
  x: number;
  y: number;
  heading: number;
  hasControl: boolean;
  on: boolean;
  numberUsers: number;
  pinned: boolean;
  spotlighted: boolean;
  handleClick: (e: React.MouseEvent) => void;
  handleMouseEnter: (e: React.MouseEvent) => void; // Use to set focus robot id
  handleMouseLeave: (e: React.MouseEvent) => void;
}

const useStyles = makeStyles({
  // style rule
  avatarContainer: (props: RobotAvatarProps) => {
    const size = 30;
    return {
      position: 'absolute',
      borderRadius: '50%',
      backgroundColor: props.spotlighted
        ? '#ff9933'
        : props.pinned
        ? '#be374a'
        : props.hasControl
        ? '#a4bc13'
        : props.on
        ? '#40b2aa'
        : '#8e8e93', // Green if has control, blue if on the robot, grey otherwise
      left: `${props.x - size / 2}px`,
      top: `${props.y - size / 2}px`,
      color: '#eee',
      height: `${size}px`,
      width: `${size}px`,
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
    };
  },
  userCountDot: {
    position: 'absolute',
    backgroundColor: 'white',
    height: '4px',
    width: '4px',
    transformOrigin: 'center',
    top: '-2px',
    left: '-2px',
    borderRadius: '50%',
  },
});

export default function RobotAvatar(props: RobotAvatarProps) {
  // How to get my own name?
  // const { room } = useVideoContext();
  // const localName = room!.localParticipant.identity;
  const classes = useStyles(props);
  const userCountDotToCenter = 12;
  const pieSlice = (2 * Math.PI) / 12;

  const toAngle = (a: number) => ((a % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);

  //<div className={classes.headingLine}></div>

  return (
    <>
      <div
        className={classes.avatarContainer}
        onClick={props.handleClick}
        onMouseEnter={props.handleMouseEnter}
        onMouseLeave={props.handleMouseLeave}
      >
        {props.spotlighted ? <StarIcon fontSize={'small'} /> : props.pinned ? <Lock fontSize={'small'}></Lock> : null}
      </div>
      <div>
        {Array.from(Array(props.numberUsers).keys()).map(ind => (
          <div
            className={classes.userCountDot}
            key={ind}
            style={{
              transform: `translate(${props.x +
                userCountDotToCenter *
                  Math.cos(toAngle(props.heading - ((props.numberUsers - 1) / 2 - ind) * pieSlice))}px,
                                                                              ${props.y +
                                                                                userCountDotToCenter *
                                                                                  Math.sin(
                                                                                    toAngle(
                                                                                      props.heading -
                                                                                        ((props.numberUsers - 1) / 2 -
                                                                                          ind) *
                                                                                          pieSlice
                                                                                    )
                                                                                  )}px)`,
            }}
          />
        ))}
      </div>
    </>
  );
}
