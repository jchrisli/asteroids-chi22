import React from 'react';
import { createStyles, makeStyles, Theme } from '@material-ui/core/styles';
import { isClassExpression } from 'typescript';
//import useVideoContext from '../../../hooks/useVideoContext/useVideoContext';

interface WorkspaceAreaProps {
  id: number;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  on: boolean;
}

const useStyles = makeStyles({
  // style rule
  workspace: (props: WorkspaceAreaProps) => {
    //const size = 30;
    let left = Math.min(props.x1, props.x2),
      top = Math.min(props.y1, props.y2),
      width = Math.max(props.x1, props.x2) - left,
      height = Math.max(props.y1, props.y2) - top;
    return {
      position: 'absolute',
      backgroundColor: 'transparent', // Green if has control, blue if on the robot, grey otherwise
      borderColor: props.on ? '#f5234aff' : '#f5234a44',
      mixBlendMode: 'hard-light',
      borderWidth: '3px',
      borderStyle: 'dashed',
      left: `${left}px`,
      top: `${top}px`,
      height: `${height}px`,
      width: `${width}px`,
    };
  },
});

export default function WorkspaceArea(props: WorkspaceAreaProps) {
  // How to get my own name?
  // const { room } = useVideoContext();
  // const localName = room!.localParticipant.identity;
  const classes = useStyles(props);

  return <div className={classes.workspace}></div>;
}
