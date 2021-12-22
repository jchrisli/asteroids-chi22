import React from 'react';
import clsx from 'clsx';
import Participant from '../Participant/Participant';
import { makeStyles, createStyles, Theme } from '@material-ui/core/styles';
import useMainParticipant from '../../hooks/useMainParticipant/useMainParticipant';
import useParticipants from '../../hooks/useParticipants/useParticipants';
import useVideoContext from '../../hooks/useVideoContext/useVideoContext';
import useSelectedParticipant from '../VideoProvider/useSelectedParticipant/useSelectedParticipant';
import { isMobile } from '../../utils';
import { Hidden } from '@material-ui/core';
import { Robot } from '../Room/Room';
import { useState } from 'react';
import { useEffect } from 'react';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    container: {
      overflowY: 'auto',
      background: 'rgb(79, 83, 85)',
      gridArea: '1 / 2 / 1 / 3',
      zIndex: 5,
      [theme.breakpoints.down('sm')]: {
        gridArea: '2 / 1 / 3 / 3',
        overflowY: 'initial',
        overflowX: 'auto',
        display: 'flex',
      },
    },
    transparentBackground: {
      background: 'transparent',
    },

    scrollContainer: {
      display: 'flex',
      justifyContent: 'center',
    },
    innerScrollContainer: {
      width: isMobile ? `calc(${theme.sidebarWidth}px)` : `calc(${theme.sidebarWidth}px - 3em)`,
      padding: isMobile ? '0px' : '1.5em 0',
      [theme.breakpoints.down('sm')]: {
        width: 'auto',
        padding: isMobile ? '0px' : `${theme.sidebarMobilePadding}px`,
        display: 'flex',
        flexWrap: isMobile ? 'wrap' : `default`,
        justifyContent: 'center',
      },
      columns: isMobile ? '2 auto' : 'default',
    },
    containerItem: {
      padding: isMobile ? '0px' : `default`,
    },
    faded: {
      opacity: 0.0,
    },
  })
);

interface ParticipantListProps {
  focusRobotId: number;
  robots: Robot[];
}

export default function ParticipantList({ focusRobotId, robots }: ParticipantListProps) {
  const classes = useStyles();
  const { room } = useVideoContext();
  const localParticipant = room!.localParticipant;
  const participants = useParticipants();
  const [selectedParticipant, setSelectedParticipant] = useSelectedParticipant();
  //const mainParticipant = useMainParticipant();
  const isRemoteParticipantScreenSharing = false;

  const localRobotId =
    isMobile && localParticipant.identity.startsWith('mobile') ? parseInt(localParticipant.identity.substr(6, 1)) : -1;
  const [localRobotUsers, setLocalRobotUsers] = useState<string[]>([]);
  useEffect(() => {
    const localRobot = robots.filter(r => r.id === localRobotId);
    if (localRobot.length > 0) {
      setLocalRobotUsers(localRobot[0].users);
    }
  }, [robots]);

  if (participants.length === 0) return null; // Don't render this component if there are no remote participants.

  return (
    <aside
      className={clsx(classes.container, {
        [classes.transparentBackground]: !isRemoteParticipantScreenSharing,
      })}
    >
      <div className={classes.scrollContainer}>
        <div className={classes.innerScrollContainer}>
          {(focusRobotId > 0
            ? [
                ...participants.filter(p => p.identity === `mobile${focusRobotId}`),
                ...participants.filter(p => p.identity !== `mobile${focusRobotId}`),
              ]
            : participants
          ).map(participant => {
            //const isSelected = participant === selectedParticipant;
            const display =
              (isMobile && localRobotUsers.indexOf(participant.identity) !== -1) ||
              (!isMobile && participant.identity.startsWith('mobile'));
            // Map particpant is displayed in the control bar
            if (!participant.identity.startsWith('map')) {
              return (
                <div
                  key={participant.sid}
                  className={clsx(classes.containerItem, {
                    [classes.faded]: !isMobile && participant.identity !== `mobile${focusRobotId}`,
                  })}
                >
                  <Participant
                    key={participant.sid}
                    participant={participant}
                    isSelected={participant === selectedParticipant}
                    onClick={() => setSelectedParticipant(participant)}
                    forceAudio={!display}
                  />
                </div>
              );
            } else return null;
          })}
        </div>
      </div>
    </aside>
  );
}
