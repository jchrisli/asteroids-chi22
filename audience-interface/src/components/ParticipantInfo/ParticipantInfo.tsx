import React from 'react';
import clsx from 'clsx';
import { createStyles, makeStyles, Theme } from '@material-ui/core/styles';
import { LocalAudioTrack, LocalVideoTrack, Participant, RemoteAudioTrack, RemoteVideoTrack } from 'twilio-video';

import AudioLevelIndicator from '../AudioLevelIndicator/AudioLevelIndicator';
import AvatarIcon from '../../icons/AvatarIcon';
import NetworkQualityLevel from '../NetworkQualityLevel/NetworkQualityLevel';
import PinIcon from './PinIcon/PinIcon';
import Typography from '@material-ui/core/Typography';

import useIsTrackSwitchedOff from '../../hooks/useIsTrackSwitchedOff/useIsTrackSwitchedOff';
import usePublications from '../../hooks/usePublications/usePublications';
import useTrack from '../../hooks/useTrack/useTrack';
import useParticipantIsReconnecting from '../../hooks/useParticipantIsReconnecting/useParticipantIsReconnecting';

import { isMobile } from '../../utils';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    container: {
      position: 'relative',
      display: 'flex',
      alignItems: 'center',
      height: 0,
      overflow: 'hidden',
      marginBottom: '0.5em',
      '& video': {
        filter: 'none',
        objectFit: 'contain !important',
      },
      borderRadius: '4px',
      border: `${theme.participantBorderWidth}px solid rgb(245, 248, 255)`,
      paddingTop: `calc(${(9 / 16) * 100}% - ${theme.participantBorderWidth}px)`,
      background: 'black',
      [theme.breakpoints.down('sm')]: {
        //height: theme.sidebarMobileHeight,
        height: `${theme.sidebarMobileHeight - 6}px`,
        width: `${((theme.sidebarMobileHeight - 6) * 16) / 9}px`,
        marginRight: '8px',
        marginBottom: '0',
        paddingTop: `${theme.sidebarMobileHeight - 2}px`,
      },
    },
    innerContainer: {
      position: 'absolute',
      top: 0,
      left: 0,
      width: '100%',
      height: '100%',
    },
    infoContainer: {
      position: 'absolute',
      zIndex: 2,
      transform: 'translateZ(2px)',
      display: 'flex',
      flexDirection: 'column',
      justifyContent: 'space-between',
      height: '100%',
      width: '100%',
      background: 'transparent',
      top: 0,
    },
    avatarContainer: {
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      background: 'black',
      position: 'absolute',
      top: 0,
      right: 0,
      bottom: 0,
      left: 0,
      zIndex: 1,
      transform: 'translateZ(1px)',
      [theme.breakpoints.down('sm')]: {
        '& svg': {
          transform: 'scale(0.7)',
        },
      },
    },
    reconnectingContainer: {
      position: 'absolute',
      top: 0,
      right: 0,
      bottom: 0,
      left: 0,
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      background: 'rgba(40, 42, 43, 0.75)',
      zIndex: 1,
      transform: 'translateZ(1px)',
    },
    identity: {
      background: 'rgba(0, 0, 0, 0.5)',
      color: 'white',
      padding: '0.18em 0.3em',
      margin: 0,
      display: 'flex',
      alignItems: 'center',
    },
    infoRowBottom: {
      display: 'flex',
      justifyContent: 'space-between',
      position: 'absolute',
      bottom: 0,
      left: 0,
    },
    typeography: {
      color: 'white',
      [theme.breakpoints.down('sm')]: {
        fontSize: '0.75rem',
      },
    },
    hideParticipant: {
      display: 'none',
    },
    cursorPointer: {
      cursor: 'pointer',
    },
    fillSpace: {
      width: '300px',
      height: '300px',
      borderStyle: 'none',
    },
  })
);

interface ParticipantInfoProps {
  participant: Participant;
  children: React.ReactNode;
  onClick?: () => void;
  isSelected?: boolean;
  isLocalParticipant?: boolean;
  hideParticipant?: boolean;
}

export default function ParticipantInfo({
  participant,
  onClick,
  isSelected,
  children,
  isLocalParticipant,
  hideParticipant,
}: ParticipantInfoProps) {
  const publications = usePublications(participant);

  const audioPublication = publications.find(p => p.kind === 'audio');
  const videoPublication = publications.find(p => p.trackName.includes('camera'));

  const isVideoEnabled = Boolean(videoPublication);

  const videoTrack = useTrack(videoPublication);
  const isVideoSwitchedOff = useIsTrackSwitchedOff(videoTrack as LocalVideoTrack | RemoteVideoTrack);

  const audioTrack = useTrack(audioPublication) as LocalAudioTrack | RemoteAudioTrack | undefined;
  const isParticipantReconnecting = useParticipantIsReconnecting(participant);

  const classes = useStyles();

  return (
    <div
      className={clsx(classes.container, {
        [classes.hideParticipant]: hideParticipant,
        [classes.cursorPointer]: Boolean(onClick),
        [classes.fillSpace]: isLocalParticipant && isMobile,
      })}
      onClick={onClick}
      data-cy-participant={participant.identity}
    >
      <div className={classes.infoContainer}>
        <NetworkQualityLevel participant={participant} />
        <div className={classes.infoRowBottom}>
          <span className={classes.identity}>
            <AudioLevelIndicator audioTrack={audioTrack} />
            <Typography variant="body2" className={classes.typeography} component="span">
              {participant.identity}
              {isLocalParticipant && ' (You)'}
            </Typography>
          </span>
        </div>
        <div>{isSelected && <PinIcon />}</div>
      </div>
      <div className={classes.innerContainer}>
        {(!isVideoEnabled || isVideoSwitchedOff) && (
          <div className={classes.avatarContainer}>
            <AvatarIcon />
          </div>
        )}
        {isParticipantReconnecting && (
          <div className={classes.reconnectingContainer}>
            <Typography variant="body1" className={classes.typeography}>
              Reconnecting...
            </Typography>
          </div>
        )}
        {children}
      </div>
    </div>
  );
}
