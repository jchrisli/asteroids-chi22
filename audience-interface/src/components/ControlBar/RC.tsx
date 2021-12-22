import { styled } from '@material-ui/core';
import React, { useState } from 'react';
import { makeStyles, createStyles, Theme } from '@material-ui/core/styles';
import TurnLeft from '../../icons/TurnLeft';
import TurnRight from '../../icons/TurnRight';
import GoForward from '../../icons/GoForward';
import GoBackwards from '../../icons/GoBackwards';
import clsx from 'clsx';

interface RCProps {
  onForward: (e: React.MouseEvent) => void;
  onBackward: (e: React.MouseEvent) => void;
  onLeft: (e: React.MouseEvent) => void;
  onRight: (e: React.MouseEvent) => void;
  onStop: (e: React.MouseEvent) => void;
}

const IconContainer = styled('div')({
  display: 'flex',
  justifyContent: 'center',
  // flexGrow:1,
  // alignItems: 'stretch',

  padding: '6px',
  // resizeMode: 'contain'
  // marginRight: '0.3em',
});

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    containerItem: {
      display: 'flex',
      justifyContent: 'center',
      // flexGrow:1,
      // alignItems: 'stretch',
      width: '25%',
      height: '25%',
      aspectRatio: '1',
      padding: '6px',
    },
    //guideContainter: {
    //display: 'flex',
    //justifyContent: 'center',
    //width: '100%',
    //}
    containerItemPressed: {
      transform: 'scale(0.95)',
    },
  })
);

export default function RC(props: RCProps) {
  const classes = useStyles();
  const [onLeftPressed, setOnLeftPressed] = useState(false);
  const [onRightPressed, setOnRightPressed] = useState(false);
  const [onForwardPressed, setOnForwardPressed] = useState(false);
  const [onBackwardPressed, setOnBackwardPressed] = useState(false);

  const leftPressed = (e: React.MouseEvent) => {
    setOnLeftPressed(true);
    props.onLeft(e);
  };

  const rightPressed = (e: React.MouseEvent) => {
    setOnRightPressed(true);
    props.onRight(e);
  };

  const forwardPressed = (e: React.MouseEvent) => {
    setOnForwardPressed(true);
    props.onForward(e);
  };

  const backwardPressed = (e: React.MouseEvent) => {
    setOnBackwardPressed(true);
    props.onBackward(e);
  };

  const stop = (e: React.MouseEvent) => {
    setOnLeftPressed(false);
    setOnRightPressed(false);
    setOnForwardPressed(false);
    setOnBackwardPressed(false);
    props.onStop(e);
  };

  return (
    <>
      <IconContainer>
        <div
          className={clsx(classes.containerItem, onForwardPressed ? classes.containerItemPressed : null)}
          onMouseDown={forwardPressed}
          onMouseUp={stop}
        >
          <GoForward />
        </div>
      </IconContainer>
      <IconContainer>
        <div
          className={clsx(classes.containerItem, onLeftPressed ? classes.containerItemPressed : null)}
          onMouseDown={leftPressed}
          onMouseUp={stop}
        >
          <TurnLeft />
        </div>
        <div
          className={clsx(classes.containerItem, onBackwardPressed ? classes.containerItemPressed : null)}
          onMouseDown={backwardPressed}
          onMouseUp={stop}
        >
          <GoBackwards />
        </div>
        <div
          className={clsx(classes.containerItem, onRightPressed ? classes.containerItemPressed : null)}
          onMouseDown={rightPressed}
          onMouseUp={stop}
        >
          <TurnRight />
        </div>
      </IconContainer>
    </>
  );
}
