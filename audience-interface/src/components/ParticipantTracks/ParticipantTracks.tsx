import React from 'react';
import { Participant, Track } from 'twilio-video';
import Publication from '../Publication/Publication';
import usePublications from '../../hooks/usePublications/usePublications';

interface ParticipantTracksProps {
  participant: Participant;
  videoOnly?: boolean;
  videoPriority?: Track.Priority | null;
  isLocalParticipant?: boolean;
  forceAudio?: boolean;
}

/*
 *  The object model for the Room object (found here: https://www.twilio.com/docs/video/migrating-1x-2x#object-model) shows
 *  that Participant objects have TrackPublications, and TrackPublication objects have Tracks.
 *
 *  The React components in this application follow the same pattern. This ParticipantTracks component renders Publications,
 *  and the Publication component renders Tracks.
 */

export default function ParticipantTracks({
  participant,
  videoOnly,
  videoPriority,
  isLocalParticipant,
  forceAudio,
}: ParticipantTracksProps) {
  const publications = usePublications(participant);

  let filteredPublications;
  const rotateVideo = participant.identity.startsWith('mobile');
  const audioOnly = forceAudio ? true : false;

  if (false && publications.some(p => p.trackName.includes('screen'))) {
    filteredPublications = publications.filter(p => !p.trackName.includes('camera'));
  } else {
    filteredPublications = publications.filter(p => !p.trackName.includes('screen'));
  }

  return (
    <>
      {filteredPublications.map(publication => (
        <Publication
          key={publication.kind}
          publication={publication}
          participant={participant}
          isLocalParticipant={isLocalParticipant}
          videoOnly={videoOnly}
          videoPriority={videoPriority}
          rotate={rotateVideo}
          forceAudio={audioOnly}
        />
      ))}
    </>
  );
}