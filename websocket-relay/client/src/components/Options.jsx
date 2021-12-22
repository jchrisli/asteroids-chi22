import React, {useContext, useState} from 'react';

import {Button, TextField, Grid, Typography, Container, Paper} from '@material-ui/core';
import {makeStyles} from '@material-ui/core/styles';
import {CopyToClipboard} from 'react-copy-to-clipboard';
import {Assignment, Phone, PhoneDisabled} from '@material-ui/icons'

const useStyles = makeStyles((theme) => ({
    root: {
      display: 'flex',
      flexDirection: 'column',
    },
    gridContainer: {
      width: '100%',
      [theme.breakpoints.down('xs')]: {
        flexDirection: 'column',
      },
    },
    container: {
      width: '600px',
      margin: '35px 0',
      padding: 0,
      [theme.breakpoints.down('xs')]: {
        width: '80%',
      },
    },
    margin: {
      marginTop: 20,
    },
    padding: {
      padding: 20,
    },
    paper: {
      padding: '10px 20px',
      border: '2px solid black',
    },
}));

const Options = ({children}) => {
    // const {me, receive, name, setName, ended} = useContext(SocketContext);
    // const [idToCall, setIdToCall] = useState('');
    const classes = useStyles();
    return (
        <Container className={classes.container}>
            <Paper elevation={10} className={classes.paper}>
                <form className={classes.root} noValidate autoComplete="off">
                    <Grid container className={classes.gridContainer }>
                        <Grid item xs={12} md={6} className={classes.padding}>
                            <Typography gutterBottom variant="h6">
                                Account info
                            </Typography>
                            {/* <TextField label="Name" value={name} onChange={(e)=>setName(e.target.value)} fullWidth/>
                            {console.log(me)}
                            <CopyToClipboard text={me} className={classes.margin}>
                                <Button variant="contained" color="primary" fullWidth startIcon={<Assignment fontSize="large"></Assignment>}>
                                    Copy your ID
                                </Button>
                            </CopyToClipboard> */}
                        </Grid>
                        {/* <Grid item xs={12} md={6} className={classes.padding}>
                            <Typography gutterBottom variant="h6">
                                Call
                            </Typography>
                            <TextField label="ID to call" value={idToCall} onChange={(e)=>setIdToCall(e.target.value)} fullWidth/>
                            {receive && !ended ? (
                                <Button 
                                variant="contained" 
                                color="secondary" 
                                // fullWidth onClick={leave}
                                className={classes.margin}
                                startIcon={<PhoneDisabled fontSize="large" />}>
                                    Hang up
                                </Button>
                            ) : (
                                <Button 
                                variant="contained" color="primary"
                                className={classes.margin} 
                                fullWidth
                                // onClick={() => createPeer(idToCall)}
                                startIcon={<Phone fontSize="large"/>}>
                                    call
                                </Button>
                            )
                            }
                        </Grid> */}
                    </Grid>
                </form>          
                {children} 
            </Paper>
            
        </Container>
    )
}

export default Options