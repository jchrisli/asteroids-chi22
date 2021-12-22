import React from 'react';
import ParticipantInfo from '../ParticipantInfo/ParticipantInfo';
import ParticipantTracks from '../ParticipantTracks/ParticipantTracks';
import { Participant as IParticipant } from 'twilio-video';

interface ParticipantProps {
  participant: IParticipant;
  videoOnly?: boolean;
  onClick?: () => void;
  isSelected?: boolean;
  isLocalParticipant?: boolean;
  hideParticipant?: boolean;
  forceAudio?: boolean;
}

export default function Participant({
  participant,
  videoOnly,
  onClick,
  isSelected,
  isLocalParticipant,
  hideParticipant,
  forceAudio,
}: ParticipantProps) {
  const audioOnly = forceAudio ? true : false;
  if (!audioOnly)
    return (
      <ParticipantInfo
        participant={participant}
        onClick={onClick}
        isSelected={isSelected}
        isLocalParticipant={isLocalParticipant}
        hideParticipant={hideParticipant}
      >
        <ParticipantTracks participant={participant} videoOnly={videoOnly} isLocalParticipant={isLocalParticipant} />
      </ParticipantInfo>
    );
  else {
    return (
      <ParticipantTracks
        participant={participant}
        videoOnly={videoOnly}
        isLocalParticipant={isLocalParticipant}
        forceAudio={true}
      />
    );
  }
}
